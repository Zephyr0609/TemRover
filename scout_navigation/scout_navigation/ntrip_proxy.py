"""Local NTRIP proxy that asks the caster for NTRIP 1.0, so corrections arrive as plain RTCM.

The team's RTK bridge requests NTRIP 2.0 and writes the response body straight to the receiver.
VicPos then answers with HTTP chunked transfer encoding, and every chunk's length line lands in the
middle of the RTCM frames, so the receiver throws them away. Pointed at this proxy, the bridge's
request is forwarded without its Ntrip-Version header; the caster replies "ICY 200 OK" and streams
unframed RTCM, which the bridge already handles. Credentials only pass through.
"""
import argparse
import socket
import threading


def pipe(source, destination):
    """Copies bytes one way until either side closes, then closes both."""
    try:
        while True:
            data = source.recv(4096)
            if not data:
                break
            destination.sendall(data)
    except OSError:
        pass
    finally:
        for side in (source, destination):
            try:
                side.close()
            except OSError:
                pass


def as_ntrip_1(request):
    """The client's request header as NTRIP 1.0: HTTP/1.0 and no Ntrip-Version line."""
    lines = request.split(b'\r\n')
    lines[0] = lines[0].replace(b'HTTP/1.1', b'HTTP/1.0')
    return b'\r\n'.join(line for line in lines if not line.lower().startswith(b'ntrip-version'))


def serve(client, caster_host, caster_port):
    request = b''
    while b'\r\n\r\n' not in request:
        data = client.recv(4096)
        if not data:
            client.close()
            return
        request += data
    header, _, rest = request.partition(b'\r\n\r\n')
    caster = socket.create_connection((caster_host, caster_port), timeout=10)
    caster.settimeout(None)
    caster.sendall(as_ntrip_1(header) + b'\r\n\r\n' + rest)
    threading.Thread(target=pipe, args=(client, caster), daemon=True).start()
    pipe(caster, client)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--listen-port', type=int, default=2101)
    parser.add_argument('--caster-host', default='gnss.vicpos.com.au')
    parser.add_argument('--caster-port', type=int, default=2020)
    arguments, _ = parser.parse_known_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', arguments.listen_port))
    server.listen(2)
    print(f'NTRIP 1.0 proxy on 127.0.0.1:{arguments.listen_port} -> '
          f'{arguments.caster_host}:{arguments.caster_port}', flush=True)
    while True:
        client, _ = server.accept()
        threading.Thread(target=serve, args=(client, arguments.caster_host, arguments.caster_port),
                         daemon=True).start()


if __name__ == '__main__':
    main()
