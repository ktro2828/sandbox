enum IpAddressType {
    V4,
    V6,
}

struct IpAddress {
    kind: IpAddressType,
    address: String,
}

// 型指定してデータを直接挿入できる
enum IpAddr {
    V4(String),
    V6(String),
}

impl IpAddress {
    fn ping(&self, ip: &String) {
        println!("ping from {} to {}", self.address, ip);
    }

    fn get_kind(&self) -> &IpAddressType {
        return &self.kind;
    }

    fn get_address(&self) -> &String {
        return &self.address;
    }
}

fn main() {
    let lo: IpAddress = IpAddress {
        kind: IpAddressType::V4,
        address: String::from("127.0.0.1"),
    };
    lo.ping(&String::from("127.0.0.2"));
    let _kind = lo.get_kind();
    println!("address: {}", lo.get_address());

    let home = IpAddr::V4(String::from("127.0.0.1"));
    let loopback = IpAddr::V6(String::from("::1"));
}
