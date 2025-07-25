use std::{cell::RefCell, rc::Rc, sync::Arc, vec};

pub trait Far {
    fn far(&self) -> String;
}

#[derive(Debug)]
struct Foo<T>(T);

impl<T> Far for Foo<T> {
    fn far(&self) -> String {
        String::from("far from foo")
    }
}

#[derive(Debug)]
struct Bar<T> {
    pub foo: RefCell<Foo<T>>,
}

impl<T> Bar<T> {
    pub fn new(foo: T) -> Self {
        Self {
            foo: RefCell::new(Foo(foo)),
        }
    }

    pub fn from_foo(foo: Foo<T>) -> Self {
        Self {
            foo: RefCell::new(foo),
        }
    }
}

fn update_bar_of_foo<T>(bar: &mut Bar<T>)
where
    T: std::ops::AddAssign<i32>,
{
    bar.foo.borrow_mut().0 += 1;
}

impl<T> Far for Bar<T> {
    fn far(&self) -> String {
        String::from("far from bar")
    }
}

fn show_far<T>(value: &T)
where
    T: Far, // T is trait bound
{
    println!("{:?}", value.far());
}

fn show_far_dyn(value: &dyn Far) {
    println!("{:?}", value.far());
}

fn main() {
    // dynamic dispatch using Box<T>
    let boxes: Vec<Box<dyn Far>> = vec![Box::new(Foo(1)), Box::new(Bar::new(1))]; // store values in a vector as unique_ptr for a single thread
    for v in boxes.iter() {
        show_far_dyn(v.as_ref());
    }

    // dynamic dispatch using Rc<T>
    let rcs: Vec<Rc<dyn Far>> = vec![Rc::new(Foo(1)), Rc::new(Bar::new(1))]; // store values in a vector as shared_ptr for a single thread
    for v in rcs.iter() {
        show_far_dyn(v.as_ref());
    }

    // dynamic dispatch using Arc<T>
    let arcs: Vec<Arc<dyn Far>> = vec![Arc::new(Foo(1)), Arc::new(Bar::new(1))]; // store values in a vector as shared_ptr for multiple threads
    for v in arcs.iter() {
        show_far_dyn(v.as_ref());
    }

    // static dispatch
    let foo = Foo(1);
    show_far(&foo);
    println!("foo = {}", foo.0);

    // by using RefCell, we can mutate the value inside the RefCell
    let mut bar = Bar::from_foo(foo);
    println!("bar.foo = {}", bar.foo.borrow().0);
    update_bar_of_foo(&mut bar);
    println!("bar.foo = {}", bar.foo.borrow().0);
}
