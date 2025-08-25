pub use crate::can::data::egs_52::*;


pub struct Egs52Can {
    gs218: Gs218H,
    gs338: Gs338H,
    gs418: Gs418H
}

impl Egs52Can {
    pub fn new() -> Self {
        todo!()
        //Gs218H::default();
    }
}