// 핀 번호
const int left = 13;
const int center = 12;
const int right = 11;

// 주차장 상태
int remain = 0;
int flags[3] = {true, true, true};
int last_btn_status[3] = {HIGH, HIGH, HIGH};

void setup()
{
  Serial.begin(115200);
  pinMode(left, INPUT_PULLUP);
  pinMode(center, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
}

void loop()
{
  int current_btn_status[3] = {digitalRead(left), digitalRead(center), digitalRead(right)};

  for (int i = 0; i < 3; i++)
  {
    if (current_btn_status[i] == LOW && last_btn_status[i] == HIGH)
    {
      flags[i] = !flags[i];
    }
    last_btn_status[i] = current_btn_status[i];
    if (i == 2)
    {
      Serial.print(flags[i]);
    }
    else
    {
      Serial.print(flags[i]);
    }
  }
  remain = flags[0] + flags[1] + flags[2];
  Serial.println(remain);
  delay(100);
}