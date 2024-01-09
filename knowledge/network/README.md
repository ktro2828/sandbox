# TCP/IP

## Glossary

### ネットマスク

IPアドレスは、「ネットワークアドレス」(=サブネット)と「ホストアドレス」の２つの部分に分解できる。このとき、ネットワークアドレスをフィルタするためのマスク用の32bit数値を「ネットマスク」(=サブネットマスク)という。

<img src="https://teltonika-networks.com/cdn/extras/18115/netmask-in-article-3-840xAuto.webp">

## Setup

```shell
$ sudo apt update
$ sudo apt -y install \
    bash \
    coreutils \
    grep \
    iproute2 \
    iputils-ping \
    traceroute \
    tcpdump \
    bind9-dnsutils \
    dnsmasq-base \
    netcat-openbsd \
    python3 \
    curl \
    wget \
    iptables \
    procps \
    isc-dhcp-client
```
