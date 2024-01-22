CREATE TABLE robot_status (
    id INT AUTO_INCREMENT PRIMARY KEY,
    status VARCHAR(20) NOT NULL,
    request VARCHAR(20),
    destination VARCHAR(20),
    battery_level FLOAT NOT NULL
)