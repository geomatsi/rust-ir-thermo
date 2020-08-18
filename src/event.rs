use heapless::consts::*;
use heapless::spsc::Queue;

#[derive(Debug, Clone, Copy)]
pub enum Event {
    Button1,
    Button2,
    Enter,
    Repeat,
}

pub struct EventQueue {
    queue: Queue<Event, U4>,
    stats: u32,
}
impl EventQueue {
    pub fn new() -> Self {
        Self {
            queue: Queue(heapless::i::Queue::new()),
            stats: 0,
        }
    }

    pub fn enqueue(&mut self, ev: Event) {
        self.queue.enqueue(ev).ok();
        self.stats = self.stats.saturating_add(1);
    }

    pub fn dequeue(&mut self) -> Option<Event> {
        self.queue.dequeue()
    }

    pub fn is_empty(&mut self) -> bool {
        self.queue.is_empty()
    }

    pub fn get_stats(&mut self) -> u32 {
        self.stats
    }

    pub fn reset_stats(&mut self) {
        self.stats = 0;
    }
}

impl Default for EventQueue {
    fn default() -> Self {
        Self::new()
    }
}
