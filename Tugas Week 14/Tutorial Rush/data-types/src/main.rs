struct Student {
    name: String,
    level: u8,
    remote: bool
}

// Tuple structs. Just the data types in this one
struct Grades(char,char,char, f32);

fn main() {
    println!("Hello, {} {}!","arif","satorus");

    let mut age = 21;
    let birth_year=2003;

    println!("aku {} tahun", age);

    let birth_year = birth_year -1;
    age=22;

    println!("aku  sekarang {} tahun", age);
    println!("aku lahir {}", birth_year);
    
    let arif_age: u32 = 14;
    println!("arif ku {} tahun",arif_age);
    let float: f32 = 4.0;
    println!("1 * 2 = {}",1 * 2);
    let is_bigger_num = 2 < 4;
    println! ("jika 2<4: {}",is_bigger_num);
    let first_char: char='w';
    let last_char :char='l';
    let second_char ='i';
    let my_name ="arif";
    println! ("{} karakter pertama,{} adalah karakter terakhr,{}adalah karakter kedua dari namaku,{}",first_char,last_char,second_char,my_name);
    let my_dog = ("Toby", 15, false);

    println!("nama naga ku adalah {}, dia {} tahun, apakah dia hidup? {}", my_dog.0, my_dog.1, my_dog.2);
    let student_1 = Student{
        name: String::from("arif satorus"),
        remote: true,
        level: 5
    };

    let grades = Grades('A','A','B',3.5);
    println!("{}, is a level {} programmer. Does he work remotely: {}",
    student_1.name, student_1.level, student_1.remote);

println!("{},{},{},GPA = {}", grades.0, grades.1, grades.2, grades.3);
}