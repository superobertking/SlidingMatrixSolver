#![feature(is_sorted)]
use std::env;
use std::io;
use std::io::prelude::*;

const USAGE: &str = "solver <dimension>";

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Operation {
    Nop = 0,
    Up = 1,
    Left = 2,
    Right = 3,
    Down = 4,
}
use Operation::*;

impl Default for Operation {
    #[inline(always)]
    fn default() -> Self {
        Self::Nop
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Step {
    pub op: Operation,
    pub jump: u8,
    pub pos: u8, // col if Up | Down, row if Left | Right
}

fn main() {
    let dim = get_arg();
    assert!(2 <= dim && dim <= 15, "Dimension not supported!");

    let map = get_map();
    assert_eq!(map.len(), (dim * dim) as usize, "Dimension not match!");

    let res = astar::do_astar(dim, map);
    output_res(dim, &res);
}

fn output_res(dim: u8, res: &[Step]) {
    println!("Solution need {} steps:", res.len() - 1);

    for step in res.iter().skip(1) {
        match step.op {
            Up | Down => println!("Col {} go {:?} {} times;", step.pos + 1, step.op, step.jump),
            Left | Right => println!("Row {} go {:?} {} times;", step.pos + 1, step.op, step.jump),
            Nop => {}
        }
    }

    println!("Submission sequence:");
    let mut iter = res.iter().skip(1).peekable();
    while let Some(step) = iter.next() {
        match step.op {
            Right => print!("{},0,{}", step.pos, step.jump),
            Left => print!("{},0,{}", step.pos, dim - step.jump),
            Down => print!("{},1,{}", step.pos, step.jump),
            Up => print!("{},1,{}", step.pos, dim - step.jump),
            Nop => {}
        }
        if iter.peek().is_some() {
            print!(",");
        } else {
            println!();
        }
    }

    println!("Done.")
}

fn get_arg() -> u8 {
    env::args().nth(1).expect(USAGE).parse().expect(USAGE)
}

fn get_map() -> Vec<u8> {
    let mut map = Vec::new();
    for line in io::stdin().lock().lines() {
        map.extend(
            line.unwrap()
                .split_whitespace()
                .map(|x| x.parse::<u8>().unwrap()),
        );
    }
    map
}

mod astar {
    use super::{Operation, Step};
    use std::cmp::{Ord, Ordering};
    use Operation::*;

    #[derive(Clone, PartialEq, Eq, Debug)]
    struct State {
        cost: usize, // steps.len() + heuristic value
        map: Vec<u8>,
        steps: Vec<Step>,
    }

    impl State {
        #[inline]
        fn last_step(&self) -> Step {
            *self.steps.last().unwrap()
        }
    }

    impl PartialOrd for State {
        #[inline]
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    /// Customized Ord for min-heap
    impl Ord for State {
        #[inline]
        fn cmp(&self, other: &Self) -> Ordering {
            match other.cost.cmp(&self.cost) {
                // cost same, but steps longer
                Ordering::Equal => self.steps.len().cmp(&other.steps.len()),
                ord => ord,
            }
        }
    }

    fn get_all_directions(dim: u8) -> Vec<(Operation, u8)> {
        let mut res = Vec::new();

        // half up, half down
        let halfu = (dim - 1) / 2;
        let halfd = (dim - 1) - halfu;

        res.extend((1..=halfu).map(|x| (Left, x)));
        res.extend((1..=halfd).map(|x| (Right, x)));
        res.extend((1..=halfu).map(|x| (Up, x)));
        res.extend((1..=halfd).map(|x| (Down, x)));

        assert!(res.len() & 1 == 0);
        res
    }

    #[inline]
    fn get_next_directions(dirs: &[(Operation, u8)], last_op: Operation) -> &[(Operation, u8)] {
        let half_len = dirs.len() / 2;
        match last_op {
            Up | Down => &dirs[..half_len],
            Left | Right => &dirs[half_len..],
            Nop => &dirs,
        }
    }

    pub fn do_astar(dim: u8, map: Vec<u8>) -> Vec<Step> {
        use std::collections::BinaryHeap;

        // a min heap due to customized Ord
        let mut queue = BinaryHeap::new();

        // init state
        queue.push(Box::new(State {
            cost: calc_h(dim, &map),
            map: map,
            steps: vec![Step::default()],
        }));

        let all_dirs = get_all_directions(dim);

        let mut iterations = 0;

        loop {
            if false {
                if iterations < 5 {
                    println!("{:?}", queue);
                } else {
                    panic!("halt");
                }
            }

            iterations += 1;

            let last_state = queue.pop().unwrap();

            if true {
                if iterations % 20000 == 0 {
                    println!("{} {}", last_state.steps.len(), last_state.cost);
                    println!("{}", queue.len());
                    println!("{:?}", last_state.steps);
                    println!("{:?}", last_state.map);
                }
            }

            let next_dirs = get_next_directions(&all_dirs, last_state.last_step().op);

            for pos in 0..dim {
                for (op, jump) in next_dirs {
                    let (op, jump) = (*op, *jump);
                    let mut next_state = last_state.clone();

                    // push current step
                    next_state.steps.push(Step { op, jump, pos });

                    // update map
                    if op == Up {
                        for i in 0..dim - jump {
                            next_state.map[(i * dim + pos) as usize] =
                                last_state.map[((i + jump) * dim + pos) as usize];
                        }
                        for i in 0..jump {
                            next_state.map[((dim - jump + i) * dim + pos) as usize] =
                                last_state.map[(i * dim + pos) as usize];
                        }
                    } else if op == Down {
                        for i in 0..jump {
                            next_state.map[(i * dim + pos) as usize] =
                                last_state.map[((dim - jump + i) * dim + pos) as usize];
                        }
                        for i in 0..dim - jump {
                            next_state.map[((i + jump) * dim + pos) as usize] =
                                last_state.map[(i * dim + pos) as usize];
                        }
                    } else if op == Left {
                        next_state.map[(pos * dim) as usize..(pos * dim + dim) as usize]
                            .rotate_left(jump as usize);
                    } else if op == Right {
                        next_state.map[(pos * dim) as usize..(pos * dim + dim) as usize]
                            .rotate_right(jump as usize);
                    } else {
                        unreachable!();
                    }

                    // check if is target
                    if next_state.map.is_sorted() {
                        println!("[d] Search finished. Stats:");
                        dbg!(queue.len());
                        return next_state.steps;
                    }
                    // otherwise push to queue and continue searching

                    // TODO: may consider use multi-stage searches

                    // update cost
                    next_state.cost = next_state.steps.len() + calc_h(dim, &next_state.map);

                    if next_state.cost <= last_state.cost + 5 {
                        queue.push(next_state);
                    }
                }
            }
        }
    }

    fn calc_h(dim: u8, map: &[u8]) -> usize {
        assert_eq!((dim * dim) as usize, map.len());
        let mut res = 0;

        for ci in 0..dim {
            let mut slot = vec![false; dim as usize];
            for cj in 0..dim {
                let ty = (map[(ci * dim + cj) as usize] - 1) % dim;
                let mut dy = dim + ty - cj;
                if dy >= dim {
                    dy -= dim;
                }
                slot[dy as usize] = true;
            }
            res += slot.iter().skip(1).map(|&b| b as usize).sum::<usize>();
        }
        for cj in 0..dim {
            let mut slot = vec![false; dim as usize];
            for ci in 0..dim {
                let tx = (map[(ci * dim + cj) as usize] - 1) / dim;
                let mut dx = dim + tx - ci;
                if dx >= dim {
                    dx -= dim;
                }
                slot[dx as usize] = true;
            }
            res += slot.iter().skip(1).map(|&b| b as usize).sum::<usize>();
        }

        res
    }
}
