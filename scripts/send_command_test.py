#!/usr/bin/env python3

import argparse
import json
import sys
import urllib.error
import urllib.parse
import urllib.request


def build_url(host, port, site_id, device_id, command_mode):
    query = urllib.parse.urlencode(
        {
            "siteId": site_id,
            "deviceId": device_id,
            "commandMode": command_mode,
        }
    )
    return f"http://{host}:{port}/sendCommand?{query}"


def main():
    parser = argparse.ArgumentParser(
        description="Send a test /sendCommand request to indooruav_http_server."
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP server host")
    parser.add_argument("--port", type=int, default=20000, help="HTTP server port")
    parser.add_argument("--site-id", type=int, default=11, help="siteId value")
    parser.add_argument("--device-id", type=int, default=1, help="deviceId value")
    parser.add_argument(
        "--command-mode", type=int, default=1, help="commandMode value"
    )
    parser.add_argument(
        "--timeout", type=float, default=3.0, help="request timeout in seconds"
    )
    args = parser.parse_args()

    url = build_url(
        args.host, args.port, args.site_id, args.device_id, args.command_mode
    )
    print(f"Request URL: {url}")

    request = urllib.request.Request(url=url, method="GET")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=args.timeout) as response:
            body = response.read().decode("utf-8", errors="replace")
            print(f"HTTP status: {response.status}")
            print(f"Response body: {body}")
            try:
                payload = json.loads(body)
            except json.JSONDecodeError:
                return 0

            result_code = payload.get("resultCode")
            if result_code is not None:
                print(f"Parsed resultCode: {result_code}")
            return 0
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        print(f"HTTP error: {exc.code}")
        print(f"Response body: {body}")
        return 1
    except urllib.error.URLError as exc:
        print(f"Request failed: {exc}")
        return 2
    except Exception as exc:  # pragma: no cover
        print(f"Unexpected error: {exc}")
        return 3


if __name__ == "__main__":
    sys.exit(main())
