#![feature(is_sorted)]
#![feature(const_generics)]
#![allow(incomplete_features)]
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

    let res = match dim {
        2 => astar::do_astar::<2, 4>(map),
        3 => astar::do_astar::<3, 9>(map),
        4 => astar::do_astar::<4, 16>(map),
        5 => astar::do_astar::<5, 25>(map),
        6 => astar::do_astar::<6, 36>(map),
        _ => unimplemented!("Algorithm not yet implemented for this dimension."),
    }
    .expect("Wrong algorithm: no solution found!");

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
    use std::collections::BinaryHeap;
    use Operation::*;

    #[derive(Clone)]
    struct State<const DIM: usize, const DIM2: usize> {
        cost: usize, // steps.len() + heuristic value
        steps: Vec<Step>,
        map: [u8; DIM2],
    }

    impl<const DIM: usize, const DIM2: usize> Default for State<DIM, DIM2> {
        fn default() -> Self {
            State {
                cost: 0,
                steps: vec![],
                map: [0; DIM2],
            }
        }
    }

    impl<const DIM: usize, const DIM2: usize> State<DIM, DIM2> {
        #[inline]
        fn last_step(&self) -> Step {
            *self.steps.last().unwrap()
        }
    }

    impl<const DIM: usize, const DIM2: usize> PartialEq for State<DIM, DIM2> {
        #[inline]
        fn eq(&self, other: &Self) -> bool {
            self.cost == other.cost && self.map[..] == other.map[..] && self.steps == other.steps
        }
    }

    impl<const DIM: usize, const DIM2: usize> Eq for State<DIM, DIM2> {}

    impl<const DIM: usize, const DIM2: usize> PartialOrd for State<DIM, DIM2> {
        #[inline]
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    /// Customized Ord for min-heap
    impl<const DIM: usize, const DIM2: usize> Ord for State<DIM, DIM2> {
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
    fn get_next_direction_pairs(
        dirs: &[(Operation, u8)],
        last_op: Operation,
    ) -> (&[(Operation, u8)], &[(Operation, u8)]) {
        let half_len = dirs.len() / 2;
        match last_op {
            Up | Down => (&dirs[half_len..], &dirs[..half_len]),
            Left | Right => (&dirs[..half_len], &dirs[half_len..]),
            Nop => (&[], &dirs),
        }
    }

    pub fn do_astar<const DIM: usize, const DIM2: usize>(in_map: Vec<u8>) -> Option<Vec<Step>> {
        // a min heap according to customized Ord
        let mut queue = BinaryHeap::new();
        // Boxing State seems to have a drop on performance.

        assert_eq!(DIM * DIM, in_map.len());

        // init state
        let mut init_state = State::default();
        init_state.cost = calc_h::<DIM>(&in_map);
        init_state
            .map
            .iter_mut()
            .zip(in_map.iter())
            .map(|(a, b)| *a = *b)
            .for_each(drop);
        init_state.steps = vec![Step::default()];

        queue.push(init_state);

        let all_dirs = get_all_directions(DIM as u8);

        let mut iterations = 0;

        while let Some(last_state) = queue.pop() {
            if false {
                if iterations < 5 {
                    // println!("{:?}", queue);
                } else {
                    panic!("halt");
                }
            }

            iterations += 1;

            if true {
                if iterations % 20000 == 0 {
                    println!("{} {}", last_state.steps.len(), last_state.cost);
                    println!("{}", queue.len());
                    println!("{:?}", &last_state.steps[..]);
                    println!("{:?}", &last_state.map[..]);
                    return Some(vec![]);
                }
            }

            // next same directions, next perpendicular direcitons
            let (next_same, next_perp) =
                get_next_direction_pairs(&all_dirs, last_state.last_step().op);

            // h funciton not giving admissible results
            if true {
                // if false {
                for pos in last_state.last_step().pos + 1..DIM as u8 {
                    let res = add_next_moves::<DIM, DIM2>(pos, &mut queue, next_same, &last_state);
                    if res.is_some() {
                        return res;
                    }
                }
            }
            for pos in 0..DIM as u8 {
                let res = add_next_moves::<DIM, DIM2>(pos, &mut queue, next_perp, &last_state);
                if res.is_some() {
                    return res;
                }
            }
        }

        None
    }

    fn add_next_moves<const DIM: usize, const DIM2: usize>(
        pos: u8,
        queue: &mut BinaryHeap<State<DIM, DIM2>>,
        next_dirs: &[(Operation, u8)],
        last_state: &State<DIM, DIM2>,
    ) -> Option<Vec<Step>> {
        let dim = DIM as u8;

        for &(op, jump) in next_dirs {
            let mut next_state = last_state.clone();

            // push current step
            next_state.steps.push(Step { op, jump, pos });

            // update map
            match op {
                Up => {
                    for i in 0..dim - jump {
                        next_state.map[(i * dim + pos) as usize] =
                            last_state.map[((i + jump) * dim + pos) as usize];
                    }
                    for i in 0..jump {
                        next_state.map[((dim - jump + i) * dim + pos) as usize] =
                            last_state.map[(i * dim + pos) as usize];
                    }
                }
                Down => {
                    for i in 0..jump {
                        next_state.map[(i * dim + pos) as usize] =
                            last_state.map[((dim - jump + i) * dim + pos) as usize];
                    }
                    for i in 0..dim - jump {
                        next_state.map[((i + jump) * dim + pos) as usize] =
                            last_state.map[(i * dim + pos) as usize];
                    }
                }
                Left => {
                    next_state.map[(pos * dim) as usize..(pos * dim + dim) as usize]
                        .rotate_left(jump as usize);
                }
                Right => {
                    next_state.map[(pos * dim) as usize..(pos * dim + dim) as usize]
                        .rotate_right(jump as usize);
                }
                Nop => unreachable!(),
            }

            // check if is target
            if next_state.map.is_sorted() {
                println!("[d] Search finished. Stats:");
                dbg!(queue.len());
                return Some(next_state.steps);
            }
            // otherwise push to queue and continue searching

            // update cost
            // next_state.cost = next_state.steps.len() + calc_h(dim, &next_state.map);
            next_state.cost = next_state.steps.len() + calc_h::<DIM>(&next_state.map);

            if next_state.cost <= last_state.cost + 5 {
                queue.push(next_state);
            }
        }

        None
    }

    /// `h` function for A\* algorithms.
    ///
    /// The simple idea is as such. For each number on the map, calculate the
    /// check if it requires a horizontal or vertical move, if it does, count
    /// one for each direction. For the same line, we only count same number
    /// of jumps as one.
    ///
    /// TODO: This h funciton is not admissible.
    fn calc_h<const DIM: usize>(map: &[u8]) -> usize {
        let dim = DIM as u8;

        assert_eq!((dim * dim) as usize, map.len());
        let mut res = 0;

        for ci in 0..dim {
            let mut slot = vec![false; dim as usize];
            for cj in 0..dim {
                let n = map[(ci * dim + cj) as usize] - 1;
                let (_, ty) = (n / dim, n % dim);
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
                let n = map[(ci * dim + cj) as usize] - 1;
                let (tx, _) = (n / dim, n % dim);
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

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_h_4_finish() {
            let h = calc_h::<4>(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
            assert_eq!(h, 0);
        }

        #[test]
        fn test_h_4_one_h() {
            let h = calc_h::<4>(&[1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 9, 13, 14, 15, 16]);
            assert!(h <= 1);
        }

        #[test]
        fn test_h_4_one_v() {
            let h = calc_h::<4>(&[1, 2, 15, 4, 5, 6, 3, 8, 9, 10, 7, 12, 13, 14, 11, 16]);
            assert!(h <= 1);
        }

        #[test]
        fn test_h_4_two_perp() {
            let h = calc_h::<4>(&[1, 2, 15, 4, 5, 6, 3, 8, 10, 7, 12, 9, 13, 14, 11, 16]);
            assert!(h <= 2, "h: {}", h);
        }

        #[test]
        fn test_h_4_two_same() {
            let h = calc_h::<4>(&[5, 2, 15, 4, 9, 6, 3, 8, 13, 10, 7, 12, 1, 14, 11, 16]);
            assert!(h <= 2, "h: {}", h);
        }
    }
}
