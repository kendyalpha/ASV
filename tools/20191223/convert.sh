# !/bin/bash
# convert old format of database into new one
 sqlite3 ./dbsave1.db <<!
.headers on
.mode csv
.output gps.csv
select * from GPS;
.output controller.csv
select * from controller;
.output estimator.csv
select * from estimator;
.output planner.csv
select * from planner;
.output indicator.csv
select * from indicators;
!


ALTER TABLE team RENAME TO team_orig;
CREATE TABLE GPS(ID INTEGER PRIMARY KEY AUTOINCREMENT,  \
				DATETIME TEXT NOT NULL, \
				UTC DOUBLE \
				latitude DOUBLE \
				longitude DOUBLE \
				heading DOUBLE \
				pitch DOUBLE \
				roll DOUBLE \
				altitude DOUBLE \
				Ve DOUBLE \
				Vn DOUBLE \
				roti DOUBLE \
				Ve DOUBLE \
				Ve DOUBLE \



				);

INSERT INTO team(Name, Coach, Location) SELECT Name, Coach, City FROM team_orig;
DROP TABLE team_orig;