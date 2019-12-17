#include <sqlite_modern_cpp.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
using namespace sqlite;
using namespace std;

int main() {
  try {
    database db("test.db");

    uint8_t jim[4] = {0x01, 0x17, 0xca, 0xff};

    db << "CREATE TABLE person (_id integer primary key autoincrement not "
          "null, angle int, numbers BLOB);";

    db << "INSERT INTO person (angle, numbers) VALUES (?,?)" << 1
       << vector<uint8_t>(&jim[0], &jim[4]);
    db << "INSERT INTO person (angle, numbers) VALUES (?,?)" << 2
       << vector<uint8_t>(&jim[0], &jim[4]);
    db << "INSERT INTO person (angle, numbers) VALUES (?,?)" << 3
       << vector<uint8_t>(&jim[0], &jim[4]);

    vector<uint8_t> numbers_test;
    db << "SELECT numbers from person where ID = ?;" << 1 >> numbers_test;

    printf("%u", numbers_test[3]);
    // else {
    // db << "SELECT numbers from person where name = ?;" << "sara" >>
    // [](vector<double> numbers_sara){ for(auto e : numbers_sara) cout << e <<
    // '
    // '; cout << endl;
    //};
    //}

  } catch (sqlite_exception e) {
    cout << "Unexpected error " << e.what() << endl;
    exit(EXIT_FAILURE);
  } catch (...) {
    cout << "Unknown error\n";
    exit(EXIT_FAILURE);
  }

  cout << "OK\n";
  exit(EXIT_SUCCESS);
}
