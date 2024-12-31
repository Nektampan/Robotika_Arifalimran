fn main() {
    let probabilities = vec![0.8, 0.6, 0.9, 0.5];
    let mut best_path = None;
    let mut max_probability = 0.0;

    for (index, &prob) in probabilities.iter().enumerate() {
        if prob > max_probability {
            max_probability = prob;
            best_path = Some(index);
        }
    }

    println!("Best path index: {:?} with probability: {:.2}", best_path, max_probability);
}