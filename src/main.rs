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

fn main() {
    let dim = get_arg();
    assert!(2 <= dim && dim <= 15, "Dimension not supported!");

    let map = get_map();
    assert_eq!(map.len(), (dim * dim) as usize, "Dimension not match!");

    let res = astar::do_astar(dim, map);
    println!("{:?}", res);
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
    use super::Operation;
    use std::cmp::{Ord, Ordering};

    #[derive(Clone, PartialEq, Eq, Debug)]
    struct State {
        pos: (u8, u8),
        cost: usize, // steps.len() + heuristic value
        map: Vec<u8>,
        steps: Vec<(Operation, u8)>, // (Op, jumps)
    }

    impl State {
        #[inline]
        fn last_step(&self) -> (Operation, u8) {
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
                Ordering::Equal => other.steps.len().cmp(&self.steps.len()),
                ord => ord,
            }
        }
    }

    fn get_all_directions(dim: u8) -> Vec<(Operation, u8)> {
        use Operation::*;

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
        use Operation::*;

        let half_len = dirs.len() / 2;
        match last_op {
            Up | Down => &dirs[..half_len],
            Left | Right => &dirs[half_len..],
            Nop => &dirs,
        }
    }

    pub fn do_astar(dim: u8, map: Vec<u8>) -> Vec<(Operation, u8)> {
        use std::collections::BinaryHeap;
        use Operation::*;

        // a min heap due to customized Ord
        let mut queue = BinaryHeap::new();

        // init state
        queue.push(Box::new(State {
            pos: (0, 0),
            cost: calc_h(dim, &map),
            map: map,
            steps: vec![(Nop, 0)],
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

            let next_dirs = get_next_directions(&all_dirs, last_state.last_step().0);

            for (op, jump) in next_dirs {
                let (op, jump) = (*op, *jump);
                let mut next_state = last_state.clone();

                // push current step
                next_state.steps.push((op, jump));

                // old positions
                let (px, py) = last_state.pos;
                // update map
                if op == Up {
                    for i in 0..dim - jump {
                        next_state.map[(i * dim + py) as usize] =
                            last_state.map[((i + jump) * dim + py) as usize];
                    }
                    for i in 0..jump {
                        next_state.map[((dim - jump + i) * dim + py) as usize] =
                            last_state.map[(i * dim + py) as usize];
                    }
                } else if op == Down {
                    for i in 0..jump {
                        next_state.map[(i * dim + py) as usize] =
                            last_state.map[((dim - jump + i) * dim + py) as usize];
                    }
                    for i in 0..dim - jump {
                        next_state.map[((i + jump) * dim + py) as usize] =
                            last_state.map[(i * dim + py) as usize];
                    }
                } else if op == Left {
                    next_state.map[(px * dim) as usize..(px * dim + dim) as usize]
                        .rotate_left(jump as usize);
                } else if op == Right {
                    next_state.map[(px * dim) as usize..(px * dim + dim) as usize]
                        .rotate_right(jump as usize);
                } else {
                    unreachable!();
                }

                // check if is target
                if next_state.map.is_sorted() {
                    return next_state.steps;
                }
                // otherwise push to queue and continue searching

                // update cost
                next_state.cost = next_state.steps.len() + calc_h(dim, &next_state.map);

                // update position
                let (p, delta) = match op {
                    Up => (&mut next_state.pos.0, dim - jump),
                    Down => (&mut next_state.pos.0, jump),
                    Left => (&mut next_state.pos.1, dim - jump),
                    Right => (&mut next_state.pos.1, jump),
                    _ => unreachable!(),
                };
                *p += delta;
                if *p >= dim {
                    *p -= dim;
                }

                queue.push(next_state);
            }
        }
    }

    fn calc_h(dim: u8, map: &[u8]) -> usize {
        assert_eq!((dim * dim) as usize, map.len());
        let mut res = 0;
        let mut x = 0;
        for ci in 0..dim {
            for cj in 0..dim {
                let (tx, ty) = ((map[x] - 1) / dim, (map[x] - 1) % dim);

                res += (tx != ci) as usize;
                res += (ty != cj) as usize;
                // res += if tx > ci { tx - ci } else { ci - tx } as usize;
                // res += if ty > cj { ty - cj } else { cj - ty } as usize;

                x += 1;
            }
        }

        res
    }

    /* #[inline]
    fn same_cost(a: Operation, b: Operation) -> bool {
        // get discriminant
        let (ad, bd) = (a as u32, b as u32);
        // specially desinated discriminant
        ad + bd == 5
    } */
}
