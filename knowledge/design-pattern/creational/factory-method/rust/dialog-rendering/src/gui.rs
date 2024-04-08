pub mod html_gui;
pub mod windows_gui;

/**
 * Interface of Product & Creator
 */

// Interface of Product
pub trait Button {
    fn render(&self);
    fn on_click(&self);
}

// Interface of Creator
pub trait Dialog {
    // The factory method. It must be overridden with a concrete implementation.
    fn create_button(&self) -> Box<dyn Button>;

    fn render(&self) {
        let button = self.create_button();
        button.render();
    }

    fn refresh(&self) {
        println!("Dialog - Refresh");
    }
}
