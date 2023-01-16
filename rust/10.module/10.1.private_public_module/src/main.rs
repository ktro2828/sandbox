// `my_mod` という名前のモジュール
mod my_mod {
    // モジュール内の要素はデフォルトでプライベート
    fn private_function() {
        println!("called `my_mod::private_function()`");
    }

    // `pub`を用いてパブリックに変更
    pub fn function() {
        println!("called `my_mod::function()`");
    }

    // 同一モジュール内でプライベートなアイテムにアクセスは可能
    pub fn indirect_access() {
        print!("called `my_mod::indirect_access()`, that\n> ");
        private_function();
    }

    // モジュールはネストできる
    pub mod nested {
        pub fn function() {
            println!("called `my_mod::nested::function()`");
        }

        #[allow(dead_code)]
        fn private_function() {
            println!("called `my_mod::nested::private_function()`");
        }

        // `pub(in path)`で指定したパス内のみで参照可能
        pub(in crate::my_mod) fn public_function_in_my_mod() {
            print!("called `my_mod::nested::public_function_in_my_mod()`, that\n> ");
            public_function_in_nested();
        }

        // `pub(self)`指定でその関が所属するモジュール内(=my_mod::nested)でのみ参照可能
        pub(self) fn public_function_in_nested() {
            println!("called `my_mod::nested::public_function_in_nested()`");
        }

        // `pub(super)`指定で親モジュール(=my_mod)からでも参照可能
        pub(super) fn public_function_in_super_mod() {
            println!("called `my_mod::nested::public_function_in_super_mod()`");
        }
    }

    pub fn call_public_fuction_in_my_mod() {
        print!("called `my_mod::call_public_fuction_in_my_mod()`, that\n>");
        nested::public_function_in_my_mod();
        print!("> ");
        nested::public_function_in_super_mod();
    }

    // pub(crate)指定でcrate内で参照可能
    pub(crate) fn public_function_in_crate() {
        println!("called `my_mod::public_function_in_crate()`");
    }

    mod private_nested {
        #[allow(dead_code)]
        pub fn function() {
            println!("called `my_mod::private_nested::function()`");
        }

        #[allow(dead_code)]
        pub(crate) fn restricted_function() {
            println!("called `my_mod::private_nested::restricted_function()`");
        }
    }
}

fn function() {
    println!("called `function()`");
}

fn main() {
    // モジュールによって同名の関数を区別できる
    function();
    my_mod::function();

    // パブリックな要素なら，ネストしていてもモジュール外からアクセスできる
    my_mod::indirect_access();
    my_mod::nested::function();
    my_mod::call_public_fuction_in_my_mod();

    // pub(crate)の要素は同じクレート内ならどこからでもアクセスできる
    my_mod::public_function_in_crate();
}
