#!/bin/bash
 sqlite3 ./dbsave4.db <<!
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
