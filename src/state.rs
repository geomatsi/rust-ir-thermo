use core::fmt;

#[derive(Clone, Copy, PartialEq)]
pub enum Menu {
    Shot,
    ShotMem,
    Cont,
    ContMem,
    View,
    Stream,
}

#[derive(Clone, Copy, PartialEq)]
pub enum State {
    Select(Menu),
    Active(Menu),
    Sleep,
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let s = match *self {
            State::Select(Menu::Shot) | State::Active(Menu::Shot) => "Shot",
            State::Select(Menu::Cont) | State::Active(Menu::Cont) => "Cont",
            State::Select(Menu::ShotMem) => "ShotMem",
            State::Active(Menu::ShotMem) => "SM",
            State::Select(Menu::ContMem) => "ContMem",
            State::Active(Menu::ContMem) => "CM",
            State::Select(Menu::View) => "View",
            State::Active(Menu::View) => "VM",
            State::Select(Menu::Stream) => "Stream",
            State::Active(Menu::Stream) => "SM",
            State::Sleep => "Sleep",
        };
        write!(f, "{}", s)
    }
}

impl State {
    pub fn next(state: State) -> State {
        match state {
            State::Select(Menu::Shot) => State::Select(Menu::ShotMem),
            State::Select(Menu::ShotMem) => State::Select(Menu::Cont),
            State::Select(Menu::Cont) => State::Select(Menu::ContMem),
            State::Select(Menu::ContMem) => State::Select(Menu::View),
            State::Select(Menu::View) => State::Select(Menu::Stream),
            State::Select(Menu::Stream) => State::Select(Menu::Shot),
            _ => state,
        }
    }

    pub fn prev(state: State) -> State {
        match state {
            State::Select(Menu::Shot) => State::Select(Menu::Stream),
            State::Select(Menu::ShotMem) => State::Select(Menu::Shot),
            State::Select(Menu::Cont) => State::Select(Menu::ShotMem),
            State::Select(Menu::ContMem) => State::Select(Menu::Cont),
            State::Select(Menu::View) => State::Select(Menu::ContMem),
            State::Select(Menu::Stream) => State::Select(Menu::View),
            _ => state,
        }
    }
}
