extern crate yaml_rust;
use std::{env, fs};
use yaml_rust::{yaml::Yaml, YamlLoader};

// Define a struct that can clean up the way we load things
// from ros param server

pub struct BuffYamlUtil {
    pub yaml_path: String,
    pub yaml_data: Yaml,
}

impl BuffYamlUtil {
    pub fn read_yaml_as_string(yaml_path: &str) -> String {
        fs::read_to_string(yaml_path).expect(format!("No config in {}", yaml_path).as_str())
    }

    pub fn new(bot_name: &str) -> BuffYamlUtil {
        let robot_name = bot_name;
        let project_root = env::var("PROJECT_ROOT").expect("Project root not set");

        let yaml_path = format!(
            "{}/buffpy/data/robots/{}/nodes.yaml",
            project_root, robot_name
        );
        let yaml_string = BuffYamlUtil::read_yaml_as_string(yaml_path.as_str());

        BuffYamlUtil {
            yaml_path: yaml_path,
            yaml_data: YamlLoader::load_from_str(yaml_string.as_str()).unwrap()[0].clone(),
        }
    }

    pub fn from_self() -> BuffYamlUtil {
        let project_root = env::var("PROJECT_ROOT").expect("Project root not set");
        let self_path = format!("{}/buffpy/data/robots/self.txt", project_root);
        let robot_name = fs::read_to_string(self_path).unwrap();

        let yaml_path = format!(
            "{}/buffpy/data/robots/{}/nodes.yaml",
            project_root, robot_name
        );

        let yaml_string = BuffYamlUtil::read_yaml_as_string(yaml_path.as_str());

        BuffYamlUtil {
            yaml_path: yaml_path,
            yaml_data: YamlLoader::load_from_str(yaml_string.as_str()).unwrap()[0].clone(),
        }
    }

    pub fn default() -> BuffYamlUtil {
        let robot_name = rosrust::param("/buffbot/robot_name")
            .unwrap()
            .get::<String>()
            .unwrap();

        BuffYamlUtil::new(&robot_name.as_str())
    }

    pub fn load_string(&self, item: &str) -> String {
        self.yaml_data[item].as_str().unwrap().to_string()
    }

    pub fn load_u16(&self, item: &str) -> u16 {
        self.yaml_data[item].as_i64().unwrap() as u16
    }

    pub fn load_u128(&self, item: &str) -> u128 {
        self.yaml_data[item].as_i64().unwrap() as u128
    }

    pub fn load_string_list(&self, item: &str) -> Vec<String> {
        self.yaml_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| x.as_str().unwrap().to_string())
            .collect()
    }

    pub fn load_u8_list(&self, item: &str) -> Vec<u8> {
        self.yaml_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| x.as_i64().unwrap() as u8)
            .collect()
    }

    pub fn load_integer_matrix(&self, item: &str) -> Vec<Vec<u8>> {
        self.yaml_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| {
                x.as_vec()
                    .unwrap()
                    .iter()
                    .map(|x| x.as_i64().unwrap() as u8)
                    .collect()
            })
            .collect()
    }

    pub fn load_float_matrix(&self, item: &str) -> Vec<Vec<f64>> {
        self.yaml_data[item]
            .as_vec()
            .unwrap()
            .iter()
            .map(|x| {
                x.as_vec()
                    .unwrap()
                    .iter()
                    .map(|x| x.as_f64().unwrap())
                    .collect()
            })
            .collect()
    }
}
