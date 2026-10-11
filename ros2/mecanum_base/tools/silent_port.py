"""A pty that never sends anything (stands in for a wrong device). Counts bytes the node writes to it."""
import os
import pty
import select
import sys
import time
import tty

m, s = pty.openpty()
tty.setraw(s)
link = sys.argv[1]
secs = float(sys.argv[2])
if os.path.islink(link):
    os.unlink(link)
os.symlink(os.ttyname(s), link)
got = b''
end = time.monotonic() + secs
while time.monotonic() < end:
    r, _, _ = select.select([m], [], [], 0.1)
    if r:
        got += os.read(m, 4096)
os.unlink(link)
print(f'silent port received {len(got)} bytes: {got[:60]!r}', flush=True)
