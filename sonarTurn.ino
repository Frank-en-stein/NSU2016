void sonarTurn_left() {
  hardbreak();
  delay(10);
  dash();
  do{
    wheel(-cave_turn, cave_turn);
    wallget();
  }while(SM!=0 || SL<setpoint_wall);
  hardbreak();
  delay(10);
}

void sonarTurn_right() {
  hardbreak();
  delay(10);
  dash();
  do{
    wheel(cave_turn, -cave_turn);
    wallget();
  }while(SM!=0 || SR<setpoint_wall);
  hardbreak();
  delay(10);
}

void sonarTurn_Right90() {
  blindRight90();
  while(SR==0) {
    wallget();
    //wallfollowLeft();
  }
}

void sonarTurn_Left90() {
  blindLeft90();
  while(SL==0) {
    wallget();
    wallfollowRight();
  }
}
