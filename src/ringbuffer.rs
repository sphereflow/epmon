use core::ops::Range;
use embassy_net::tcp::TcpSocket;
use embedded_svc::io::asynch::Write;

#[derive(Debug)]
pub struct RingBuffer<const SIZE: usize> {
    buf: [u16; SIZE],
    current_ix: usize,
    transmission_ix: usize,
}

impl<const SIZE: usize> Default for RingBuffer<SIZE> {
    fn default() -> Self {
        Self {
            buf: [0; SIZE],
            current_ix: 0,
            transmission_ix: 0,
        }
    }
}

impl<const SIZE: usize> RingBuffer<SIZE> {
    pub fn push(&mut self, val: u16) {
        self.current_ix += 1;
        // make sure to not 'walk' over the transmission_ix => push it in 'front' of current_ix to
        // ensure the transmission of the whole buffer
        if self.transmission_ix == self.current_ix {
            self.transmission_ix = (self.current_ix + 1) % SIZE;
        }
        self.current_ix %= SIZE;
        self.buf[self.current_ix] = val;
    }

    pub fn get_range(&self, range: Range<usize>, buf: &mut [u16]) {
        for (ix_dest, ix_src) in range.enumerate() {
            buf[ix_dest] = self.buf[(1 + self.current_ix + ix_src) % SIZE];
        }
    }

    pub fn retransmit_whole_buffer_on_next_transmit(&mut self) {
        self.transmission_ix = (self.current_ix + 1) % SIZE;
    }

    // send diff excluding current_ix
    pub async fn send_diff(
        &mut self,
        socket: &mut TcpSocket<'_>,
        send_buf: &mut [u8],
    ) -> Result<(), embassy_net::tcp::Error> {
        let transmission_size = 2 * ((self.current_ix + SIZE - self.transmission_ix) % SIZE);
        send_buf[..4].clone_from_slice(&transmission_size.to_be_bytes());
        socket.write_all(&send_buf[..4]).await?;

        let mut sb_pos = 0;
        while self.transmission_ix != self.current_ix {
            // transfer voltage into send_buf
            // use to_le_bytes because the server can directly cast the slice to &[u16]
            send_buf[sb_pos..sb_pos + 2]
                .clone_from_slice(&self.buf[self.transmission_ix].to_le_bytes());
            sb_pos += 2;

            // if send_buf is full => send it
            if sb_pos >= send_buf.len() {
                log::info!("send_buffer: sending packet");
                socket.write_all(send_buf).await?;
                sb_pos = 0;
            }

            self.transmission_ix += 1;
            self.transmission_ix %= SIZE;
        }
        if sb_pos != 0 {
            socket.write_all(&send_buf[..sb_pos]).await?;
        }
        Ok(())
    }

    pub fn iter(&self) -> RingBufferIter<SIZE> {
        let current_iter_ix = (self.current_ix + 1) % SIZE;
        RingBufferIter {
            ring_buffer: self,
            current_iter_ix,
            num_values_produced: 0,
        }
    }
}

pub struct RingBufferIter<'a, const SIZE: usize> {
    ring_buffer: &'a RingBuffer<SIZE>,
    current_iter_ix: usize,
    num_values_produced: usize,
}

impl<'a, const SIZE: usize> Iterator for RingBufferIter<'a, SIZE> {
    type Item = u16;

    fn next(&mut self) -> Option<Self::Item> {
        if self.num_values_produced < SIZE {
            let val = self.ring_buffer.buf[self.current_iter_ix % SIZE];
            self.current_iter_ix += 1;
            self.num_values_produced += 1;
            Some(val)
        } else {
            None
        }
    }
}
