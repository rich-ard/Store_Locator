# Store Locator

## What it is?

Use of the Google Maps API and a MySQL database to render a map and list of office locations sorted by attribute.

## Changes you'll need to make:

- in "index.php", in line 183, you'll need to replace yourdatabase.yoursqlinstance.com with the URL at which your database is located;
- in "phpsqlsearch_dbinfo.php", you'll need to enter your database username, password, and the database in which the office information has been stored
- You'll need to populate your MySQL table. The code herein uses the following structure, in which the 'office_state' field is the full name of the State in which a store is located, and 'lat' and 'lng' are the latitude and longitude of the store.

  CREATE TABLE `offices` (
  
  `id` int(11) NOT NULL AUTO_INCREMENT,
  
  `office_state` char(15) NOT NULL,
  
  `street_address` varchar(60) NOT NULL,
  
  `citystatezip` varchar(60) NOT NULL,
  
  `office_phone` varchar(12) DEFAULT NULL,
  
  `lat` varchar(10) NOT NULL,
  
  `lng` varchar(10) NOT NULL,
  
  PRIMARY KEY (`id`)
  )


## Check it out:

There is an example located at http://premiumbrandt.com/atamapping which builds on the code here.


