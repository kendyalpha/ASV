
#include <sqlite_modern_cpp.h>
#include <iostream>
using namespace sqlite;
using namespace std;

int main() {
  database db("dbfile.db");
  db << "CREATE TABLE person (name TEXT, numbers BLOB);";
  db << "INSERT INTO person VALUES (?, ?)"
     << "bob" << vector<int>{5, 8, 9, 4};
  db << "INSERT INTO person VALUES (?, ?)"
     << "sara" << vector<double>{1.0, 2.0, 3.0, 4.0};

  vector<int> numbers_bob;
  db << "SELECT numbers from person where name = ?;"
     << "bob" >>
      numbers_bob;

  for (auto const &e : numbers_bob) cout << e << endl;

  db << "SELECT numbers from person where name = ?;"
     << "sara" >>
      [](vector<double> numbers_sara) {
        for (auto e : numbers_sara) cout << e << ' ';
        cout << endl;
      };
}
