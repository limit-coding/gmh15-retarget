"""Status page: actions need the page token and a local Origin; no other way in."""
import json
import urllib.error
import urllib.request

from bare_hand_teleop.live_web import LiveWeb


def post(port, body, origin):
    r = urllib.request.Request(f"http://127.0.0.1:{port}/action", data=json.dumps(body).encode(), method="POST",
                               headers={"Content-Type": "application/json", "Origin": origin})
    try:
        return urllib.request.urlopen(r, timeout=2).status
    except urllib.error.HTTPError as e:
        return e.code


def test_actions_require_token_and_origin():
    web = LiveWeb(0)
    port = web.server.server_address[1]
    web.port = port
    ok_origin = f"http://127.0.0.1:{port}"
    try:
        page = urllib.request.urlopen(f"http://127.0.0.1:{port}/").read().decode()
        assert web.token in page
        assert post(port, {"action": "arm", "token": "wrong"}, ok_origin) == 403
        assert post(port, {"action": "arm", "token": web.token}, "http://evil.example") == 403
        assert post(port, {"action": "open", "token": web.token}, "http://localhost:1234") == 200
        assert post(port, {"action": "fist", "token": web.token}, "null") == 200
        assert web.pop_actions() == ["open", "fist"]
        assert post(port, {"action": "launch", "token": web.token}, ok_origin) == 403
        assert web.pop_actions() == []
        assert post(port, {"action": "arm", "token": web.token}, ok_origin) == 200
        assert post(port, {"action": "disarm", "token": web.token}, ok_origin) == 200
        assert web.pop_actions() == ["arm", "disarm"] and web.pop_actions() == []
    finally:
        web.close()
