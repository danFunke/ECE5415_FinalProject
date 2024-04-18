void setup(){
  DDRH |= B00011000; // 6 = PH3; 7 = PH4;
  DDRE |= B00001000; // 5 = PE3;
  DDRG |= B00100000; // 4 = PG5;
}

void loop(){
  PORTH |= B00011000; // 6 = PH3; 7 = PH4;
  PORTE |= B00001000; // 5 = PE3;
  PORTG |= B00100000; // 4 = PG5;
  delayMicroseconds(1000);
  PORTH &= ~B00011000; // 6 = PH3; 7 = PH4;
  PORTE &= ~B00001000; // 5 = PE3;
  PORTG &= ~B00100000; // 4 = PG5;
  delayMicroseconds(3000);
}