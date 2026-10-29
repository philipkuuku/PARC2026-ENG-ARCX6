"""LLM intent parser: text -> the same structured command the terminal produces.

Speaks the OpenAI-compatible /chat/completions API with tool calling, so the
same code works with Groq today and a local server (Ollama, llama.cpp) later:
only base_url / model / api key change.

The model never outputs coordinates. It picks a tag ID from a fixed enum (or
asks a question); the task manager turns the ID into a pose from the registry.
"""
import json
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
- Spoken numbers: 'zero' or 'oh' = 0, 'four' = 4, 'seven' = 7, 'nine' = 9. \
After the word 'tag', 'for' almost always means 4.
- A single destination -> navigate_to. Several destinations in order -> navigate_sequence.
- 'stop', 'cancel', 'halt', 'never mind' while moving -> cancel_navigation.
- 'where can you go' / 'what places do you know' -> list_locations.
- If the request is ambiguous, names a place that is not in the list, or is not a \
navigation request, call ask_clarification with ONE short question. Do not guess.
- Keep any spoken text to one short sentence."""


def build_tools(valid_tags):
    tag_enum = sorted(int(t) for t in valid_tags)
    tag_schema = {'type': 'integer', 'enum': tag_enum}
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
            'parameters': {'type': 'object', 'properties': {}}}},
        {'type': 'function', 'function': {
            'name': 'list_locations',
            'description': 'List the places the robot knows about.',
            'parameters': {'type': 'object', 'properties': {}}}},
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
                 api_key=None, timeout=8.0, reasoning_effort=None):
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.api_key = api_key
        self.timeout = float(timeout)
        self.reasoning_effort = reasoning_effort or None   # 'low'|'medium'|'high'
        self.tools = build_tools(valid_tags)
        self.system_prompt = SYSTEM_PROMPT.format(
            tags=', '.join(str(t) for t in sorted(valid_tags)))

    # ------------------------------------------------------------------
    def parse(self, text):
        """Return a command dict, or raise LLMError."""
        body = {
            'model': self.model,
            'messages': [
                {'role': 'system', 'content': self.system_prompt},
                {'role': 'user', 'content': text},
            ],
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
            message = reply['choices'][0]['message']
        except (KeyError, IndexError, TypeError):
            raise LLMError(f'unexpected response: {str(reply)[:200]}')
        return self.to_command(message)

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
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors='replace')[:200]
            raise LLMError(f'HTTP {e.code}: {detail}')
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise LLMError(f'network error: {e}')
        except json.JSONDecodeError as e:
            raise LLMError(f'bad JSON from server: {e}')

    # ------------------------------------------------------------------
    @staticmethod
    def to_command(message):
        """Map the model's tool calls onto the terminal's command structure."""
        targets, cancel, list_tags, question = [], False, False, None
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
                    cancel = True
                elif name == 'list_locations':
                    list_tags = True
                elif name == 'ask_clarification':
                    question = str(args['question'])
                else:
                    raise LLMError(f'unknown tool {name!r}')
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                raise LLMError(f'bad arguments for {name}: {fn.get("arguments")!r}')

        if cancel:                       # safety wins over everything else
            return {'action': 'cancel'}
        if targets:
            # IDs are re-validated against the registry by the task manager.
            return {'action': 'navigate',
                    'targets': [{'type': 'tag', 'id': t} for t in targets]}
        if list_tags:
            return {'action': 'tags'}
        if question:
            return {'action': 'say', 'text': question}
        content = (message.get('content') or '').strip()
        return {'action': 'say',
                'text': content or "Sorry, I didn't catch that."}