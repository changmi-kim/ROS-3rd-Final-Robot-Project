CREATE TABLE park_system_log (
    entry_time DATETIME,
    charging_start_time DATETIME,
    charging_end_time DATETIME,
    departure_time DATETIME,
    robot_number VARCHAR(4),
    price FLOAT,
    isPayed VARCHAR(2),
    parking_location VARCHAR(4)
)