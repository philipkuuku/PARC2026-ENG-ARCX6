"""LLM intent parser: text -> the same structured command the terminal produces.

Speaks the OpenAI-compatible /chat/completions API with tool calling, so the
same code works with Groq today and a local server (Ollama, llama.cpp) later:
only base_url / model / api key change.

The model never outputs coordinates. It picks a tag ID from a fixed enum (or
asks a question); the task manager turns the ID into a pose from the registry.

The parser keeps the last few exchanges so short replies ("yes", "go ahead")
make sense. History is stored as proper tool-call / tool-result messages.
"""
import json
import threading
import time
import urllib.error
import urllib.request

DEFAULT_BASE_URL = 'https://api.groq.com/openai/v1'
# Groq's model list changes; llama-3.3-70b is now enterprise-only. gpt-oss-20b is
# self-serve with tool calling. List what your key can use with:
#   curl https://api.groq.com/openai/v1/models -H "Authorization: Bearer $GROQ_API_KEY"
DEFAULT_MODEL = 'openai/gpt-oss-20b'


class LLMError(Exception):
    """Network failure, bad HTTP status, or an unusable model reply."""


SYSTEM_PROMPT = """You control a service robot (SITO-E) in a stadium. The user speaks to you; \
the text may come from speech recognition and contain mistakes.

The robot can only drive to places marked with AprilTags. The valid tag IDs are: {tags}.

Rules:
- Act ONLY by calling tools. Never invent a tag ID that is not in the list.
- Prefer acting over asking. Ask only when the destination is missing or not in the list.
- Spoken numbers: 'one' = 1 (often heard as 'won'), 'four' = 4, 'seven' = 7, 'nine' = 9. There is no tag 0. \
After the word 'tag', 'for' almost always means 4.
- A single destination -> navigate_to. Several destinations in order -> navigate_sequence.
- The robot automatically announces every arrival and the next stop, so a request like \
'go to tag 1, say you are there, then go to tag 9' is simply navigate_sequence([1, 9]). \
Do not ask about the announcement.
- 'stop', 'cancel', 'halt', 'never mind' while moving -> cancel_navigation.
- 'where can you go' / 'what places do you know' -> list_locations.
- 'where are you' / 'what is your position' / 'which tag are you at' -> where_am_i. \
'what are you doing' / 'where are you going' / 'why did you fail' -> report_status.
- Short replies such as 'yes' or 'go ahead' refer to the previous turn of the conversation.
- If the request is ambiguous, names a place that is not in the list, or is not a \
navigation request, call ask_clarification with ONE short question. Do not guess.
- Keep any spoken text to one short sentence."""

# What we tell the model each tool "returned", so history stays well-formed.
TOOL_RESULTS = {
    'navigate_to': 'Navigation started.',
    'navigate_sequence': 'Navigation started.',
    'cancel_navigation': 'Stopped.',
    'list_locations': 'Locations shown to the user.',
    'where_am_i': 'Position reported to the user.',
    'report_status': 'Status reported to the user.',
    'ask_clarification': 'Question shown to the user.',
}


def build_tools(valid_tags):
    tag_enum = sorted(int(t) for t in valid_tags)
    tag_schema = {'type': 'integer', 'enum': tag_enum}
    no_args = {'type': 'object', 'properties': {}}
    return [
        {'type': 'function', 'function': {
            'name': 'navigate_to',
            'description': 'Drive the robot to the location marked by one AprilTag.',
            'parameters': {
                'type': 'object',
                'properties': {'tag_id': tag_schema},
                'required': ['tag_id']}}},
        {'type': 'function', 'function': {
            'name': 'navigate_sequence',
            'description': 'Visit several tag locations, in the given order.',
            'parameters': {
                'type': 'object',
                'properties': {'tag_ids': {
                    'type': 'array', 'items': tag_schema, 'minItems': 1}},
                'required': ['tag_ids']}}},
        {'type': 'function', 'function': {
            'name': 'cancel_navigation',
            'description': 'Stop the robot and cancel the current trip.',
            'parameters': no_args}},
        {'type': 'function', 'function': {
            'name': 'list_locations',
            'description': 'List the places the robot knows about.',
            'parameters': no_args}},
        {'type': 'function', 'function': {
            'name': 'where_am_i',
            'description': "Report the robot's current position and which tag location "
                           "it is at or nearest to.",
            'parameters': no_args}},
        {'type': 'function', 'function': {
            'name': 'report_status',
            'description': 'Report what the robot is doing now: current target, '
                           'remaining stops, last place reached.',
            'parameters': no_args}},
        {'type': 'function', 'function': {
            'name': 'ask_clarification',
            'description': 'Ask the user one short question when the request is '
                           'unclear or cannot be done.',
            'parameters': {
                'type': 'object',
                'properties': {'question': {'type': 'string'}},
                'required': ['question']}}},
    ]


class LLMParser:

    def __init__(self, valid_tags, base_url=DEFAULT_BASE_URL, model=DEFAULT_MODEL,
                 api_key=None, timeout=8.0, reasoning_effort=None, max_turns=4):
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.api_key = api_key
        self.timeout = float(timeout)
        self.reasoning_effort = reasoning_effort or None   # 'low'|'medium'|'high'
        self.max_turns = int(max_turns)
        self.tools = build_tools(valid_tags)
        self.system_prompt = SYSTEM_PROMPT.format(
            tags=', '.join(str(t) for t in sorted(valid_tags)))
        self._turns = []          # one list of messages per exchange
        self._call_counter = 0
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    def reset(self):
        with self._lock:
            self._turns.clear()

    def parse(self, text):
        """Return a command dict, or raise LLMError."""
        with self._lock:
            history = [m for turn in self._turns for m in turn]
        try:
            message = self._chat(text, history)
        except LLMError as e:
            # A malformed history should never lock the user out: retry clean once.
            if history and 'HTTP 400' in str(e):
                self.reset()
                message = self._chat(text, [])
            else:
                raise
        cmd = self.to_command(message)
        self._remember(self._entries(text, message))
        return cmd

    def _chat(self, text, history):
        body = {
            'model': self.model,
            'messages': ([{'role': 'system', 'content': self.system_prompt}]
                         + history + [{'role': 'user', 'content': text}]),
            'tools': self.tools,
            'tool_choice': 'auto',
            'temperature': 0,
            # Reasoning models spend tokens thinking before the tool call, so a
            # small limit can cut the reply off.
            'max_tokens': 1024,
        }
        if self.reasoning_effort:
            body['reasoning_effort'] = self.reasoning_effort
        reply = self._post('/chat/completions', body)
        try:
            return reply['choices'][0]['message']
        except (KeyError, IndexError, TypeError):
            raise LLMError(f'unexpected response: {str(reply)[:200]}')

    def _post(self, path, body):
        headers = {
            'Content-Type': 'application/json',
            # Groq sits behind Cloudflare, which can reject the default
            # "Python-urllib" user agent with a 403.
            'User-Agent': 'sito-e-nav/0.1',
        }
        if self.api_key:
            headers['Authorization'] = f'Bearer {self.api_key}'
        req = urllib.request.Request(
            self.base_url + path, data=json.dumps(body).encode(),
            headers=headers, method='POST')
        for attempt in (1, 2):
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                    return json.loads(resp.read().decode())
            except urllib.error.HTTPError as e:
                detail = e.read().decode(errors='replace')[:200]
                raise LLMError(f'HTTP {e.code}: {detail}')
            except (urllib.error.URLError, TimeoutError, OSError) as e:
                timed_out = isinstance(e, TimeoutError) or isinstance(
                    getattr(e, 'reason', None), TimeoutError)
                if attempt == 1 and not timed_out:
                    time.sleep(0.3)     # short DNS / connection hiccups are common
                    continue            # (a timeout is not retried: it is slow enough)
                raise LLMError(f'network error: {e}')
            except json.JSONDecodeError as e:
                raise LLMError(f'bad JSON from server: {e}')

    # ------------------------------------------------------------------
    def _entries(self, text, message):
        """Messages for one finished exchange, in valid tool-call format."""
        entries = [{'role': 'user', 'content': text}]
        calls = message.get('tool_calls') or []
        if not calls:
            reply = (message.get('content') or '').strip()
            entries.append({'role': 'assistant',
                            'content': reply or "Sorry, I didn't catch that."})
            return entries
        clean = []
        for c in calls:
            fn = c.get('function') or {}
            self._call_counter += 1
            clean.append({
                'id': c.get('id') or f'call_{self._call_counter}',
                'type': 'function',
                'function': {'name': fn.get('name'),
                             'arguments': fn.get('arguments') or '{}'}})
        entries.append({'role': 'assistant', 'content': message.get('content') or '',
                        'tool_calls': clean})
        for c in clean:
            entries.append({'role': 'tool', 'tool_call_id': c['id'],
                            'content': TOOL_RESULTS.get(c['function']['name'], 'Done.')})
        return entries

    def _remember(self, entries):
        with self._lock:
            self._turns.append(entries)
            del self._turns[:-self.max_turns]   # trim whole exchanges, never half

    @staticmethod
    def to_command(message):
        """Map the model's tool calls onto the terminal's command structure."""
        targets, question = [], None
        flags = set()
        for call in message.get('tool_calls') or []:
            fn = call.get('function') or {}
            name = fn.get('name')
            try:
                args = json.loads(fn.get('arguments') or '{}')
                if name == 'navigate_to':
                    targets.append(int(args['tag_id']))
                elif name == 'navigate_sequence':
                    targets.extend(int(t) for t in args['tag_ids'])
                elif name == 'cancel_navigation':
                    flags.add('cancel')
                elif name == 'list_locations':
                    flags.add('tags')
                elif name == 'where_am_i':
                    flags.add('where')
                elif name == 'report_status':
                    flags.add('status')
                elif name == 'ask_clarification':
                    question = str(args['question'])
                else:
                    raise LLMError(f'unknown tool {name!r}')
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                raise LLMError(f'bad arguments for {name}: {fn.get("arguments")!r}')

        if 'cancel' in flags:            # safety wins over everything else
            return {'action': 'cancel'}
        if targets:
            # IDs are re-validated against the registry by the task manager.
            return {'action': 'navigate',
                    'targets': [{'type': 'tag', 'id': t} for t in targets]}
        for name in ('where', 'status', 'tags'):
            if name in flags:
                return {'action': name}
        if question:
            return {'action': 'say', 'text': question}
        content = (message.get('content') or '').strip()
        return {'action': 'say',
                'text': content or "Sorry, I didn't catch that."}