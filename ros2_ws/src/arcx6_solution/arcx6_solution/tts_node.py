#!/usr/bin/env python3
"""Text-to-speech node (simulation stand-in).  Does ONE job:

    /robot/say -> Groq text-to-speech -> speaker

Topics
  subscribes  /robot/say       std_msgs/String   text to speak (queued, in order)
  publishes   /tts/speaking    std_msgs/Bool     True while speaking (the STT node
                                                 mutes its mic so it never hears us)

The voice is online for now. If the call fails (no internet) and `espeak-ng` is
installed, it speaks with that instead, so the robot is never silent.

Needs:  sudo apt install python3-sounddevice python3-numpy libportaudio2
        sudo apt install espeak-ng            # optional offline fallback
        export GROQ_API_KEY=gsk_...
"""
import json
import os
import queue
import re
import shutil
import struct
import subprocess
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import rclpy
import sounddevice as sd
from rclpy.node import Node
from std_msgs.msg import Bool, String

DEFAULT_BASE_URL = 'https://api.groq.com/openai/v1'
# English voices listed in Groq's docs: autumn, diana, hannah, austin, daniel, troy
DEFAULT_MODEL = 'canopylabs/orpheus-v1-english'


def clean_for_speech(text, limit=400):
    text = re.sub(r'[*_#`]+', ' ', text)
    return re.sub(r'\s+', ' ', text).strip()[:limit]


def chunk_text(text, limit=180):
    """Split into pieces of <= limit characters at sentence/word boundaries.

    Short pieces keep each request small and fast. Neighbouring short sentences
    are merged so a normal announcement is still one request.
    """
    pieces = []
    for sentence in re.split(r'(?<=[.!?])\s+', text.strip()):
        while len(sentence) > limit:
            cut = sentence.rfind(' ', 0, limit)
            cut = cut if cut > 0 else limit
            pieces.append(sentence[:cut].strip())
            sentence = sentence[cut:].strip()
        if sentence:
            pieces.append(sentence)
    merged = []
    for piece in pieces:
        if merged and len(merged[-1]) + 1 + len(piece) <= limit:
            merged[-1] += ' ' + piece
        else:
            merged.append(piece)
    return merged


def decode_wav(data):
    """16-bit PCM WAV bytes -> (samples[n, channels] int16, sample_rate).

    Hand-rolled instead of `wave` because streamed WAVs can carry a bogus data
    length; here we simply take whatever bytes are actually present.
    """
    if data[:4] != b'RIFF' or data[8:12] != b'WAVE':
        raise ValueError('response is not a WAV file')
    pos, channels, rate, bits = 12, None, None, None
    while pos + 8 <= len(data):
        chunk_id = data[pos:pos + 4]
        size = int.from_bytes(data[pos + 4:pos + 8], 'little')
        body = pos + 8
        if chunk_id == b'fmt ':
            _fmt, channels, rate = struct.unpack('<HHI', data[body:body + 8])
            bits = struct.unpack('<H', data[body + 14:body + 16])[0]
        elif chunk_id == b'data':
            if channels is None or bits != 16:
                raise ValueError(f'unsupported WAV (channels={channels}, bits={bits})')
            raw = data[body:body + size] if body + size <= len(data) else data[body:]
            raw = raw[:len(raw) // (2 * channels) * (2 * channels)]
            return np.frombuffer(raw, dtype='<i2').reshape(-1, channels), rate
        pos = body + size + (size & 1)
    raise ValueError('WAV has no data chunk')


class TTSNode(Node):

    def __init__(self):
        super().__init__('tts_node')
        for name, default in [
                ('base_url', DEFAULT_BASE_URL), ('model', DEFAULT_MODEL),
                ('voice', 'autumn'), ('max_chars', 180), ('timeout', 15.0),
                ('fallback_espeak', True), ('device', '')]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value   # noqa: E731

        self.key = os.environ.get('TTS_API_KEY') or os.environ.get('GROQ_API_KEY')
        self.base_url = p('base_url').rstrip('/')
        self.model, self.voice = p('model'), p('voice')
        self.max_chars, self.timeout = int(p('max_chars')), float(p('timeout'))
        device = p('device')
        self.device = int(device) if str(device).isdigit() else (device or None)
        self.espeak = (shutil.which('espeak-ng') or shutil.which('espeak')
                       ) if p('fallback_espeak') else None

        self.speaking_pub = self.create_publisher(Bool, '/tts/speaking', 10)
        self.pending = queue.Queue(maxsize=5)
        self.create_subscription(String, '/robot/say', self.on_say, 10)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(
            f'TTS ready: {self.model} / {self.voice}'
            + (' (espeak fallback available)' if self.espeak else ''))

    # -------------------------------------------------- intake
    def on_say(self, msg):
        text = clean_for_speech(msg.data)
        if not text:
            return
        try:
            self.pending.put_nowait(text)
        except queue.Full:                       # keep the newest, drop the oldest
            try:
                self.pending.get_nowait()
            except queue.Empty:
                pass
            self.pending.put_nowait(text)

    def set_speaking(self, value):
        msg = Bool()
        msg.data = value
        self.speaking_pub.publish(msg)

    # -------------------------------------------------- speaking
    def worker(self):
        while rclpy.ok():
            try:
                text = self.pending.get(timeout=0.5)
            except queue.Empty:
                continue
            self.set_speaking(True)          # mute the mic *before* any sound
            try:
                self.speak(text)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'Speaking failed: {e}')
            finally:
                self.set_speaking(False)

    def speak(self, text):
        self.get_logger().info(f'Say: "{text}"')
        for piece in chunk_text(text, self.max_chars):
            audio = self.synthesize(piece)
            if audio is not None:
                samples, rate = audio
                sd.play(samples, rate, device=self.device)
                sd.wait()
            elif self.espeak:
                self.get_logger().warn('Online TTS failed; using espeak.')
                subprocess.run([self.espeak, '-s', '160', piece],
                               timeout=30, check=False)
            else:
                self.get_logger().warn(f'Online TTS failed and no fallback: "{piece}"')

    def synthesize(self, text):
        """Groq speech endpoint -> (samples, rate), or None on failure."""
        payload = json.dumps({'model': self.model, 'input': text,
                              'voice': self.voice,
                              'response_format': 'wav'}).encode()
        headers = {'Content-Type': 'application/json',
                   'User-Agent': 'sito-e-voice/0.1'}
        if self.key:
            headers['Authorization'] = f'Bearer {self.key}'
        req = urllib.request.Request(self.base_url + '/audio/speech',
                                     data=payload, headers=headers, method='POST')
        for attempt in (1, 2):
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                    return decode_wav(resp.read())
            except urllib.error.HTTPError as e:
                detail = e.read().decode(errors='replace')[:200]
                self.get_logger().warn(f'TTS HTTP {e.code}: {detail}')
                return None
            except (urllib.error.URLError, TimeoutError, OSError) as e:
                timed_out = isinstance(e, TimeoutError) or isinstance(
                    getattr(e, 'reason', None), TimeoutError)
                if attempt == 1 and not timed_out:
                    time.sleep(0.3)
                    continue
                self.get_logger().warn(f'TTS network error: {e}')
                return None
            except ValueError as e:
                self.get_logger().warn(f'TTS gave unusable audio: {e}')
                return None


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
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
