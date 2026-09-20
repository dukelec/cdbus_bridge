Host setup
==========

The bridge presents a USB ethernet port (CDC NCM) next to its serial port,
and maps the whole bus into `fdcd::/104`. Linux, macOS and Windows 11 all
have an in-box driver for it, so nothing has to be installed and no daemon
has to run.

What does have to be done once is give that port its address. The bridge
deliberately does not hand one out: it is transparent, it has no address of
its own on the bus, and the address the host takes *is* the bridge's
identity on the bus. Pinning it here keeps the two from disagreeing.

Until this is done the serial port works exactly as it always has, so none
of it is required; the ethernet port is an addition, not a replacement.

systemd-networkd
----------------

```
sudo cp 50-cdbus-bridge.link 50-cdbus-bridge.network /etc/systemd/network/
sudo systemctl restart systemd-networkd
```

Replug the bridge. It should come up as `cdbus0` holding `fdcd::80:0` and
`fdcd::`:

```
ip -6 -br addr show dev cdbus0
```

NetworkManager
--------------

Install `50-cdbus-bridge.link` the same way so the port is named `cdbus0`,
then:

```
nmcli con add type ethernet ifname cdbus0 con-name cdbus0 \
    ipv4.method disabled \
    ipv6.method manual ipv6.addresses "fdcd::80:0/64,fdcd::/64" \
    ipv6.may-fail no
```

Addressing
----------

A CDNET address is 3 bytes, `level:net:mac`, mapped onto the last 3 bytes of
the IPv6 address, and the UDP port is the CDNET port. This is the same
mapping the `cdnet_tun` tool uses, so programs written against it work
unchanged.

| IPv6 address    | CDNET      | meaning                            |
|-----------------|------------|------------------------------------|
| `fdcd::fe`      | `00:00:fe` | level 0, mac fe                    |
| `fdcd::80:00fe` | `80:00:fe` | level 1, mac fe on our own net     |
| `fdcd::80:01fe` | `a0:01:fe` | level 1 on another net, via router |
| `fdcd::f0:00ff` | `f0:00:ff` | level 1 multicast                  |
| `fdcd::10:0`    | --         | **the bridge itself**              |

`fdcd::10:0` is the one address that never reaches the bus. It is a node the
firmware serves on its own, offering the same services the serial port's
`0xcdcd` config mode does; see the main Readme.

Note that the two are the same node on the bus, so a frame arriving on it
cannot be attributed to one of them and only one can have it: opening the
serial port takes the bus away from this interface until it is closed
again.

```python
s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
s.bind(("fdcd::80:0", 50040))
s.sendto(b"...", ("fdcd::80:00fe", 0xcdcd))
```

Port offset
-----------

A level 0 CDNET port is only 7 bits wide, and a port under 1024 needs root
to bind, so a program that wants to talk level 0 cannot simply bind the port
it wants to be. `port_offset` in the bridge config moves the host's side of
the mapping out of the way: the host sends from, and is sent to, the CDNET
port plus the offset, while the ports on the bus side stay as they are. It
is 0 by default, which changes nothing.

With it set to 20000, binding 20040 makes the program CDNET port 40:

```python
s.bind(("fdcd::80:0", 20000 + 40))
s.sendto(b"...", ("fdcd::fe", 5))   # level 0, mac fe, its config service
```

`fdcd::10:0` is shifted along with everything else, so one offset covers the
whole prefix and the bridge is reached the same way a device is. Anything
sent from below the offset is dropped and counted in `drop_fmt`, the bridge
itself included, so if an offset ever locks you out of it, set it back over
the serial port's `0xcdcd` config mode.

This is the same mapping `cdnet_tun --port-offset` does, so again, programs
written against it work unchanged.

Packet size
-----------

A CDNET packet has to fit in one CDBUS frame, and there is no fragmentation,
so **keep datagrams at 244 bytes or less** and they will always fit. The
interface MTU cannot be used to enforce this, IPv6 requires at least 1280 on
a link. Anything too large is dropped by the bridge and counted in
`drop_big`.
