#!/usr/bin/env python3
"""Speech-to-text node (simulation stand-in).  Does ONE job:

    microphone -> voice-activity detection -> Groq Whisper -> /speech/transcript

Topics
  publishes   /speech/transcript  std_msgs/String   one message per utterance
              /robot/say          std_msgs/String   only the short wake-word ack
  subscribes  /tts/speaking       std_msgs/Bool     mic is muted while the robot talks

The Whisper call is online for now (models are large for the Jetson). Swapping in
a local whisper.cpp / faster-whisper later only means changing `transcribe()`:
the topic contract stays the same.

Needs:  sudo apt install python3-sounddevice python3-numpy libportaudio2
        export GROQ_API_KEY=gsk_...
"""
import collections
import io
import json
import os
import queue
import re
import threading
import time
import urllib.error
import urllib.request
import uuid
import wave

import numpy as np
import rclpy
import sounddevice as sd
from rclpy.node import Node
from std_msgs.msg import Bool, String

DEFAULT_BASE_URL = 'https://api.groq.com/openai/v1'
SAMPLE_RATE = 16000
BLOCK = 480                                   # 30 ms of audio per block
STOP_WORDS = {'stop', 'cancel', 'halt', 'abort'}
# Whisper tends to "hear" these in silence or noise; never worth acting on.
NOISE_TEXTS = {'', 'you', 'thank you', 'thanks', 'thanks for watching',
               'bye', 'bye bye'}
# Biases Whisper toward our vocabulary (helps "four" vs "for", "won" vs "one").
DEFAULT_PROMPT = 'Tag one, tag four, tag seven, tag nine. Stop.'


def clean(text):
    """Lowercase, strip punctuation, collapse spaces (for matching only)."""
    return re.sub(r'\s+', ' ', re.sub(r"[^a-z0-9' ]+", ' ', text.lower())).strip()


def is_noise_text(text):
    return clean(text) in NOISE_TEXTS


def is_prompt_echo(text, prompt):
    """Whisper sometimes answers near-silence by repeating its own prompt.

    Acting on that would send the robot to a random tag, so drop it. A genuine
    command is shorter than, or different from, the whole prompt.
    """
    c, pc = clean(text), clean(prompt)
    if not pc or not c:
        return False
    return c == pc or pc in c or (c in pc and len(c) >= 0.8 * len(pc))


def to_wav_bytes(audio, rate=SAMPLE_RATE):
    pcm = (np.clip(audio, -1.0, 1.0) * 32767).astype('<i2')
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm.tobytes())
    return buf.getvalue()


def build_multipart(fields, file_field, filename, content, content_type):
    """multipart/form-data body, stdlib only (no `requests` dependency)."""
    boundary = uuid.uuid4().hex
    parts = []
    for key, value in fields.items():
        parts.append(
            f'--{boundary}\r\nContent-Disposition: form-data; name="{key}"'
            f'\r\n\r\n{value}\r\n'.encode())
    parts.append(
        f'--{boundary}\r\nContent-Disposition: form-data; name="{file_field}"; '
        f'filename="{filename}"\r\nContent-Type: {content_type}\r\n\r\n'.encode()
        + content + b'\r\n')
    parts.append(f'--{boundary}--\r\n'.encode())
    return b''.join(parts), f'multipart/form-data; boundary={boundary}'


class Segmenter:
    """Energy-based voice activity detection.

    Feed it 30 ms blocks; it returns a whole utterance (numpy array) once the
    speaker has been quiet for `silence_s`, or None. The noise floor is learned
    from the first second and then tracked, so the threshold follows the room.
    """

    def __init__(self, sample_rate=SAMPLE_RATE, block=BLOCK, min_rms=0.01,
                 factor=3.0, silence_s=0.8, max_s=8.0, min_speech_s=0.4,
                 preroll_s=0.3, warmup_s=1.0):
        self.block_s = block / sample_rate
        self.min_rms, self.factor = min_rms, factor
        self.silence_s, self.max_s = silence_s, max_s
        self.min_speech_s = min_speech_s
        self.preroll_len = max(1, int(preroll_s / self.block_s))
        self.warm = int(warmup_s / self.block_s)
        self.noise = None
        self.dc = None           # running estimate of the mic's DC offset
        self.last_rms = 0.0
        self.peak = 0.0          # loudest block since the debug log last read it
        self.reset()

    @property
    def threshold(self):
        return max(self.min_rms, (self.noise or 0.0) * self.factor)

    def reset(self):
        """Drop any partial utterance (the learned noise floor is kept)."""
        self.preroll = collections.deque(maxlen=self.preroll_len)
        self.utter = []
        self.in_speech = False
        self.hot = 0
        self.silence = 0.0

    def feed(self, block):
        # Some mics add a constant offset. Left in, it looks like loud room noise
        # and pushes the speech threshold far too high. Track it and subtract it.
        mean = float(np.mean(block))
        self.dc = mean if self.dc is None else 0.98 * self.dc + 0.02 * mean
        block = block - np.float32(self.dc)
        rms = float(np.sqrt(np.mean(np.square(block, dtype=np.float32))))
        self.last_rms = rms
        self.peak = max(self.peak, rms)
        if self.noise is None:
            self.noise = rms
        if self.warm > 0:                       # learn the room before listening
            self.warm -= 1
            self.noise = 0.8 * self.noise + 0.2 * rms
            return None

        loud = rms > self.threshold
        if not self.in_speech:
            if not loud:
                self.noise = 0.95 * self.noise + 0.05 * rms
            self.preroll.append(block)          # keep the start of the first word
            self.hot = self.hot + 1 if loud else 0
            if self.hot >= 2:                   # ~60 ms above threshold
                self.in_speech = True
                self.utter = list(self.preroll)
                self.silence = 0.0
            return None

        self.utter.append(block)
        self.silence = 0.0 if loud else self.silence + self.block_s
        dur = len(self.utter) * self.block_s
        if self.silence >= self.silence_s or dur >= self.max_s:
            speech = dur - self.silence
            audio = np.concatenate(self.utter)
            self.reset()
            return audio if speech >= self.min_speech_s else None
        return None


class WakeFilter:
    """Optional wake word(s). 'stop' always gets through: stopping never needs one.

    `wake_word` may list several phrases separated by commas, e.g.
    "hey robot,hi robot". Whole words only; "robots" counts, "robotics" does not.
    """

    def __init__(self, wake_word='', window_s=8.0):
        phrases = [clean(w) for w in wake_word.split(',') if clean(w)]
        self.patterns = [re.compile(rf'\b{re.escape(p)}s?\b') for p in phrases]
        self.window = float(window_s)
        self.until = 0.0

    def accept(self, text, now):
        """-> (command_text or None, wake_word_was_heard)"""
        t = clean(text)
        if set(t.split()) & STOP_WORDS:
            return text.strip(), False
        if not self.patterns:
            return text.strip(), False
        for pattern in self.patterns:
            m = pattern.search(t)
            if m:
                self.until = now + self.window      # follow-ups need no wake word
                rest = t[m.end():].strip()
                return (rest or None), True
        if now < self.until:
            self.until = now + self.window
            return text.strip(), False
        return None, False


class STTNode(Node):

    def __init__(self):
        super().__init__('stt_node')
        for name, default in [
                ('base_url', DEFAULT_BASE_URL), ('model', 'whisper-large-v3-turbo'),
                ('language', 'en'), ('prompt', DEFAULT_PROMPT), ('device', ''),
                ('wake_word', ''), ('wake_window', 8.0), ('ack_text', 'Yes?'),
                ('min_rms', 0.01), ('noise_factor', 3.0), ('silence_s', 0.8), ('max_utterance_s', 8.0),
                ('min_speech_s', 0.4), ('mute_tail_s', 0.6), ('timeout', 10.0),
                ('debug_levels', False)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value   # noqa: E731

        self.key = os.environ.get('STT_API_KEY') or os.environ.get('GROQ_API_KEY')
        self.base_url = p('base_url').rstrip('/')
        self.model, self.language, self.prompt = p('model'), p('language'), p('prompt')
        self.timeout = float(p('timeout'))
        self.ack_text = p('ack_text')
        self.mute_tail = float(p('mute_tail_s'))
        self.debug = bool(p('debug_levels'))
        self.wake = WakeFilter(p('wake_word'), p('wake_window'))
        self.segmenter = Segmenter(
            min_rms=p('min_rms'), factor=p('noise_factor'), silence_s=p('silence_s'),
            max_s=p('max_utterance_s'), min_speech_s=p('min_speech_s'))

        self.tts_speaking = False
        self.speaking_since = 0.0
        self.mute_until = 0.0

        self.pub = self.create_publisher(String, '/speech/transcript', 10)
        self.say_pub = self.create_publisher(String, '/robot/say', 10)
        self.create_subscription(Bool, '/tts/speaking', self.on_speaking, 10)

        threading.Thread(target=self.listen_loop, args=(p('device'),),
                         daemon=True).start()

    # -------------------------------------------------- muting while TTS talks
    def on_speaking(self, msg):
        self.tts_speaking = bool(msg.data)
        now = time.monotonic()
        if msg.data:
            self.speaking_since = now
        else:
            self.mute_until = now + self.mute_tail

    def muted(self):
        now = time.monotonic()
        # A TTS node that crashed mid-sentence must not leave us deaf forever.
        if self.tts_speaking and now - self.speaking_since < 30.0:
            return True
        return now < self.mute_until

    # -------------------------------------------------- audio capture
    def listen_loop(self, device):
        blocks = queue.Queue(maxsize=200)

        def callback(indata, frames, time_info, status):
            try:
                blocks.put_nowait(indata[:, 0].copy())
            except queue.Full:
                pass

        if isinstance(device, str):
            device = int(device) if device.isdigit() else (device or None)
        try:
            stream = sd.InputStream(samplerate=SAMPLE_RATE, blocksize=BLOCK,
                                    channels=1, dtype='float32', device=device,
                                    callback=callback)
            stream.start()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f'Cannot open the microphone: {e}. List devices with: '
                "python3 -c 'import sounddevice as sd; print(sd.query_devices())' "
                'then pass -p device:=<index>.')
            return

        self.get_logger().info(
            'Listening (calibrating the room for ~1 s; stay quiet).')
        last_debug = 0.0
        try:
            while rclpy.ok():
                try:
                    block = blocks.get(timeout=0.2)
                except queue.Empty:
                    continue
                if self.muted():
                    self.segmenter.reset()         # drop the robot's own voice
                    continue
                audio = self.segmenter.feed(block)
                if self.debug and time.monotonic() - last_debug > 1.0:
                    last_debug = time.monotonic()
                    seg = self.segmenter
                    self.get_logger().info(
                        f'peak={seg.peak:.4f} noise={(seg.noise or 0.0):.4f} '
                        f'threshold={seg.threshold:.4f} dc={(seg.dc or 0.0):+.4f} '
                        f'{"<-- HEARD" if seg.peak > seg.threshold else ""}')
                    seg.peak = 0.0
                if audio is not None:
                    self.on_utterance(audio)
        finally:
            stream.stop()
            stream.close()

    # -------------------------------------------------- one finished utterance
    def on_utterance(self, audio):
        text = self.transcribe(audio)
        if text is None:
            return
        text = text.strip()
        if is_noise_text(text) or is_prompt_echo(text, self.prompt):
            self.get_logger().debug(f'Ignoring noise transcript: {text!r}')
            return
        command, woke = self.wake.accept(text, time.monotonic())
        if command is None:
            if woke and self.ack_text:             # just the wake word: say "Yes?"
                msg = String()
                msg.data = self.ack_text
                self.say_pub.publish(msg)
            if woke:
                self.get_logger().info('Wake word heard.')
            else:
                self.get_logger().info(f'Heard, but no wake word (ignored): "{text}"')
            return
        self.get_logger().info(f'Heard: "{command}"')
        msg = String()
        msg.data = command
        self.pub.publish(msg)

    # -------------------------------------------------- the online model
    def transcribe(self, audio):
        """WAV -> text via Groq's OpenAI-compatible endpoint. None on failure."""
        fields = {'model': self.model, 'response_format': 'json', 'temperature': '0'}
        if self.language:
            fields['language'] = self.language
        if self.prompt:
            fields['prompt'] = self.prompt
        body, ctype = build_multipart(
            fields, 'file', 'speech.wav', to_wav_bytes(audio), 'audio/wav')
        headers = {'Content-Type': ctype, 'User-Agent': 'sito-e-voice/0.1'}
        if self.key:
            headers['Authorization'] = f'Bearer {self.key}'
        req = urllib.request.Request(
            self.base_url + '/audio/transcriptions', data=body,
            headers=headers, method='POST')
        for attempt in (1, 2):
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                    return json.loads(resp.read().decode()).get('text', '')
            except urllib.error.HTTPError as e:
                detail = e.read().decode(errors='replace')[:200]
                self.get_logger().warn(f'Transcription HTTP {e.code}: {detail}')
                return None
            except (urllib.error.URLError, TimeoutError, OSError) as e:
                timed_out = isinstance(e, TimeoutError) or isinstance(
                    getattr(e, 'reason', None), TimeoutError)
                if attempt == 1 and not timed_out:
                    time.sleep(0.3)                # brief DNS/connection hiccups
                    continue
                self.get_logger().warn(f'Transcription network error: {e}')
                return None
            except json.JSONDecodeError as e:
                self.get_logger().warn(f'Transcription returned bad JSON: {e}')
                return None


def main(args=None):
    if not (os.environ.get('STT_API_KEY') or os.environ.get('GROQ_API_KEY')):
        print('Set GROQ_API_KEY first:  export GROQ_API_KEY=gsk_...')
        return
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()