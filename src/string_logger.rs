use heapless::String;
use static_cell::ConstStaticCell;

pub static LOG: ConstStaticCell<String<256>> = ConstStaticCell::new(String::new());
static STRING_LOGGER: StringLogger = StringLogger;

pub fn init_string_logger() {
    log::set_logger(&STRING_LOGGER).unwrap();
}

struct StringLogger;

impl log::Log for StringLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        if let Some(message) = record.file() {
            loop {
                if let Some(log) = LOG.try_take() {
                    log.clear();
                    let _ = log.push_str(message);
                    break;
                }
            }
        }
    }

    fn flush(&self) {}
}
