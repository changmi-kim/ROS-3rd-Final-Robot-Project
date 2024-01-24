CREATE TABLE park_system_log (
    id INT AUTO_INCREMENT PRIMARY KEY,
    car_number VARCHAR(16),
    entry_time DATETIME,
    charging_start_time DATETIME,
    charging_end_time DATETIME,
    departure_time DATETIME,
    robot_number INT,
    price INT,
    isPayed VARCHAR(2),
    connector VARCHAR(4),
    FOREIGN KEY (robot_number) REFERENCES robot_status(id)
);