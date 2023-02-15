use std::time::Instant;

pub struct PageBuffer {
    pub data: [u8; 64],
    pub seek_ptr: usize,
    pub update_flag: bool,
    pub timestamp: Instant,
}

impl PageBuffer {
    pub fn new() -> PageBuffer {
        PageBuffer {
            data: [0; 64],
            seek_ptr: 0,
            update_flag: false,
            timestamp: Instant::now(),
        }
    }

    pub fn seek(&mut self, set: Option<usize>) -> u8 {
        self.seek_ptr = set.unwrap_or(self.seek_ptr);
        let tmp = self.data[self.seek_ptr];

        if self.seek_ptr >= 63 {
            self.seek_ptr = 0;
        } else {
            self.seek_ptr += 1;
        }

        tmp
    }

    pub fn put(&mut self, value: u8) {
        self.data[self.seek_ptr] = value;
        if self.seek_ptr < 63 {
            self.seek_ptr += 1;
        } else {
            self.seek_ptr = 0;
        }
    }

    pub fn puts(&mut self, data: Vec<u8>) {
        for d in data {
            self.put(d);
        }
    }

    pub fn check_of(&self, n: usize) -> bool {
        if 64 - self.seek_ptr >= n {
            return false;
        }
        true
    }

    pub fn reset(&mut self) {
        self.data = [0u8; 64];
        self.seek_ptr = 0;
        self.update_flag = false;
        self.timestamp = Instant::now();
    }

    pub fn print_buffer(&self) {
        let mut data_string: String = String::new();

        let mut i = 0;

        for u in self.data {
            data_string.push_str(&(u.to_string() + "\t"));

            if (i + 1) % 16 == 0 && i != 0 {
                data_string.push_str("\n");
            }
            i += 1;
        }
        data_string.push_str("\n==========\n");
        println!("{}", data_string);
    }
}

/// buffer for byte packet storage
///  stores the data, timestamp of the last access
///  and a flag for updates.
pub struct ByteBuffer {
    pub data: Vec<u8>,      // data packet
    pub timestamp: Instant, // for time sensitive things
    pub update_flag: bool,  // an update flag for updates
}

impl ByteBuffer {
    pub fn new(n: usize) -> ByteBuffer {
        /*
            Create a new buffer.
        */
        ByteBuffer {
            data: vec![0; n],
            update_flag: false,
            timestamp: Instant::now(),
        }
    }

    pub fn validate_index(&self, idx: usize) {
        if idx >= self.data.len() {
            panic!("Invalid index for operation {} < {}", idx, self.data.len());
        }
    }

    pub fn get(&mut self, idx: usize) -> u8 {
        /*
            Read a value in the buffer.
        */
        self.validate_index(idx);
        self.data[idx]
    }

    pub fn put(&mut self, idx: usize, value: u8) {
        /*
            Write a value in the buffer.
        */
        self.validate_index(idx);
        self.data[idx] = value;
    }

    pub fn get_i32(&mut self, idx: usize) -> i32 {
        /*
            Read an i32 from the buffer.
        */
        self.validate_index(idx);
        self.validate_index(idx + 3);
        i32::from_be_bytes([
            self.data[idx],
            self.data[idx + 1],
            self.data[idx + 2],
            self.data[idx + 3],
        ])
    }

    pub fn get_u32(&mut self, idx: usize) -> u32 {
        /*
            Read an i32 from the buffer.
        */
        self.validate_index(idx);
        self.validate_index(idx + 3);
        u32::from_be_bytes([
            self.data[idx],
            self.data[idx + 1],
            self.data[idx + 2],
            self.data[idx + 3],
        ])
    }

    pub fn put_i32(&mut self, idx: usize, value: i32) {
        /*
            Write an i32 to the buffer.
        */
        self.puts(idx, value.to_be_bytes().to_vec());
    }

    pub fn get_float(&mut self, idx: usize) -> f64 {
        /*
            Read an f32 from the buffer.
            return as f64 just because.
        */
        self.validate_index(idx);
        self.validate_index(idx + 3);
        f32::from_be_bytes([
            self.data[idx],
            self.data[idx + 1],
            self.data[idx + 2],
            self.data[idx + 3],
        ]) as f64
    }

    pub fn get_floats(&mut self, idx: usize, n: usize) -> Vec<f64> {
        /*
            Read an f32 from the buffer.
            return as f64 just because.
        */
        self.data[idx..idx + (4 * n)]
            .chunks_exact(4)
            .map(|x| f32::from_be_bytes(x.try_into().unwrap()) as f64)
            .collect()
    }

    pub fn put_float(&mut self, idx: usize, value: f64) {
        /*
            Write an f64 to the buffer.
            actually writes as f32
        */
        self.puts(idx, (value as f32).to_be_bytes().to_vec());
    }

    pub fn put_floats(&mut self, idx: usize, values: Vec<f64>) {
        /*
            Write an f64 to the buffer.
            actually writes as f32
        */
        values
            .iter()
            .enumerate()
            .for_each(|(i, v)| self.put_float((4 * i as usize) + idx, *v));
    }

    pub fn gets(&mut self, idx: usize, n: usize) -> Vec<u8> {
        /*
            Read a vec of values from the buffer.
        */
        self.validate_index(idx);
        self.validate_index(idx + n - 1);
        self.data[idx..idx + n].to_vec()
    }

    pub fn puts(&mut self, idx: usize, data: Vec<u8>) {
        /*
            Write a vec of values to the buffer.
        */
        self.validate_index(idx);
        self.validate_index(idx + data.len() - 1);
        self.data[idx..idx + data.len()].copy_from_slice(&data);
    }

    pub fn reset(&mut self) {
        /*
            Reset the buffer.
        */
        self.data = vec![0u8; self.data.len()];
        self.update_flag = false;
        self.timestamp = Instant::now();
    }

    pub fn print_data(&self) {
        /*
            Print the buffer.
        */
        let mut data_string: String = "\n\n==========\n".to_string();

        self.data.iter().enumerate().for_each(|(i, u)| {
            data_string.push_str(format!("{:#X}\t", u).as_str());

            if (i + 1) % 16 == 0 && i != 0 {
                data_string.push_str("\n");
            }
        });

        println!("{}", data_string);
    }
}
