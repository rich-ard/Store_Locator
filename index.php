
<html>
<head>
<title>Store Locator</title>

<!--Begin Google Maps API-->

<script src="http://maps.google.com/maps/api/js?sensor=false"
            type="text/javascript"></script>

<script type="text/javascript">
    //<![CDATA[
    var map;
    var markers = [];
    var infoWindow;
    var locationSelect;

    function load() {
      map = new google.maps.Map(document.getElementById("map"), {
	// The following latitude and longitude centers the intial map on Kalamazoo, MI.
	// Change it at your leisure.
        center: new google.maps.LatLng(43.258569, -85.798141),
        zoom: 5,
        mapTypeId: 'roadmap',
        mapTypeControlOptions: {style: google.maps.MapTypeControlStyle.DROPDOWN_MENU}
      });
      infoWindow = new google.maps.InfoWindow();

      locationSelect = document.getElementById("locationSelect");
      locationSelect.onchange = function() {
        var markerNum = locationSelect.options[locationSelect.selectedIndex].value;
        if (markerNum != "none"){
          google.maps.event.trigger(markers[markerNum], 'click');
        }
      };
   }

   function searchLocations() {
     var address = document.getElementById("addressInput").value;
     var geocoder = new google.maps.Geocoder();
     geocoder.geocode({address: address}, function(results, status) {
       if (status == google.maps.GeocoderStatus.OK) {
        searchLocationsNear(results[0].geometry.location);
       } else {
	 // The variable "radius" defined in line 70 below constrains the search to offices within 200 miles.
	 // If no stores are found within that radius, the following popup is displayed:
         alert('You entered the address:\n\n' + address + '\n\nWe did not find any offices within two hundred miles of your selected location. Please try again.');
       }
     });
   }

   function clearLocations() {
     infoWindow.close();
     for (var i = 0; i < markers.length; i++) {
       markers[i].setMap(null);
     }
     markers.length = 0;

     locationSelect.innerHTML = "";
     var option = document.createElement("option");
     option.value = "none";
     option.innerHTML = "Click here for offices by proximity:";
     locationSelect.appendChild(option);
   }

   function searchLocationsNear(center) {
     clearLocations(); 

     //The Google Maps API, by default, has the following variable 'radius'
     //filled by a form (in the below commented-out line 69; here it has been replaced by a 200-mile radius in line 70. 
     //var radius = document.getElementById('radiusSelect').value;
     var radius = 200;
     var searchUrl = 'phpsqlsearch_genxml.php?lat=' + center.lat() + '&lng=' + center.lng() + '&radius=' + radius;
     downloadUrl(searchUrl, function(data) {
       var xml = parseXml(data);
       var markerNodes = xml.documentElement.getElementsByTagName("marker");
       var bounds = new google.maps.LatLngBounds();
       for (var i = 0; i < markerNodes.length; i++) {
         var name = markerNodes[i].getAttribute("name");
         var address = markerNodes[i].getAttribute("address");
         var directionsAddress = encodeURIComponent(address);
         var office_phone = markerNodes[i].getAttribute("office_phone");
         var office_mgr = markerNodes[i].getAttribute("office_mgr");
         var mgr_email = markerNodes[i].getAttribute("mgr_email");
         var distance = parseFloat(markerNodes[i].getAttribute("distance"));
         var latlng = new google.maps.LatLng(
              parseFloat(markerNodes[i].getAttribute("lat")),
              parseFloat(markerNodes[i].getAttribute("lng")));

         createOption(name, distance, i);
         createMarker(latlng, name, address, office_phone, mgr_email, office_mgr, directionsAddress);
         bounds.extend(latlng);
       }
       map.fitBounds(bounds);
       locationSelect.style.visibility = "visible";
       locationSelect.onchange = function() {
         var markerNum = locationSelect.options[locationSelect.selectedIndex].value;
         google.maps.event.trigger(markers[markerNum], 'click');
       };
      });
    }
  
    function createMarker(latlng, name, address, office_phone, mgr_email, office_mgr, directionsAddress) {
      var html = '<b>' + name + '</b> <br>' + address + '<br>' + 'Phone: ' + office_phone + '<br> Contact manager: <a href="mailto:' + mgr_email  + '">' + office_mgr + '</a><br>' + '<a href=http://www.google.com/maps/dir/current+position/' + directionsAddress + ' target="_blank">Directions' + '</a><br>';
      var marker = new google.maps.Marker({
        map: map,
        position: latlng
      });
      google.maps.event.addListener(marker, 'click', function() {
        infoWindow.setContent(html);
        infoWindow.open(map, marker);
      });
      markers.push(marker);
    }

    function createOption(name, distance, num) {
      var option = document.createElement("option");
      option.value = num;
      option.innerHTML = name + " (" + distance.toFixed(1) + " miles)";
      locationSelect.appendChild(option);
    }

    function downloadUrl(url, callback) {
      var request = window.ActiveXObject ?
          new ActiveXObject('Microsoft.XMLHTTP') :
          new XMLHttpRequest;

      request.onreadystatechange = function() {
        if (request.readyState == 4) {
          request.onreadystatechange = doNothing;
          callback(request.responseText, request.status);
        }
      };

      request.open('GET', url, true);
      request.send(null);
    }

    function parseXml(str) {
      if (window.ActiveXObject) {
        var doc = new ActiveXObject('Microsoft.XMLDOM');
        doc.loadXML(str);
        return doc;
      } else if (window.DOMParser) {
        return (new DOMParser).parseFromString(str, 'text/xml');
      }
    }

    function doNothing() {}

    //]]>
  </script>

<!--End Google Maps API-->

</head>

<body onload="load()">

         
        	<h1>Locations</h1><br />

                Enter a location below and click "Search" to find the five closest offices. <br /><br />

                Clicking on each marker on the map will show information about each office and methods for contacting the local staff.<br /><br />
       
                <input type="text" id="addressInput" size="20" onfocus="if(this.value == 'City, State or Zip Code') { this.value = ''; }" value="City, State or Zip Code"/>

                <input type="button" onclick="searchLocations()" value="Search"/><br /><br />

                Above the map is a drop-down menu listing offices by their distance from your chosen location; scroll down for a list of offices by location.

	<!--Here begins the PHP code that creates the list of offices; the large Google Map is
	    contained herein as well in order to render the page correctly. -->
        
<?php

	// Build a list of states which links to each's position on the page
	
	require("phpsqlsearch_dbinfo.php");

	// Opens a connection to a mySQL server
	$connection=mysql_connect ('yourdatabase.yoursqlinstance.com', $username, $password);
	if (!$connection) {
	  die("Not connected : " . mysql_error());
	}

	// Set the active mySQL database
	$db_selected = mysql_select_db($database, $connection);
	if (!$db_selected) {
	  die ("Can\'t connect to database : " . mysql_error());
	}
	
	// Search the rows in the offices table for unique States in which offices are located
	$stateQuery = 'SELECT DISTINCT office_state AS state FROM offices';

	$stateResult = mysql_query($stateQuery);

	// If the query completed without errors, return an HTML list of the offices
	// which links to the state header for each of the lists of offices; otherwise, display
	// the error that prevented it.
	if ($stateResult) {
	 
	  echo "We have offices in the following states: ";
          $num_rows = mysql_num_rows($stateResult);
          $i = 1;
	  while($rows = mysql_fetch_assoc($stateResult)) {


	    if ($num_rows > $i) { 
	      echo "<a href=\"#" . $rows['state'] . "\">" . $rows['state'] . "</a>, ";
	      $i++;
            }
  	    else { echo "and <a href=\"#" . $rows['state'] . "\">" . $rows['state'] . "</a>."; }
	  }
	  mysql_data_seek($stateResult, 0);	  
	}
	else echo "An error occurred: " . mysql_error();

	// ...and here, the Google Map is rendered.

	echo "<select id=\"locationSelect\" style=\"width:100%;visibility:hidden\">
	      <option>xx</option></select><br /><br />
              <div id=\"map\" style=\"width: 670px; height: 400px\"></div>";


	// Search the rows in the offices table for office information to populate the 
	// office locations list/table
	$officeQuery = 'SELECT office_state,
			street_address,
			citystatezip,
			office_phone FROM offices 
			ORDER BY office_name';

	$officeResult = mysql_query($officeQuery);

	// If the query completed without errors, return a series of tables bookmarked by state
	// such that the links from the previous query are connected to each; otherwise, 
	// display the error that prevented it.

	if ($officeResult) {

	  // First, we need to pull the distinct State list again:
	  $stateResult2 = mysql_query($stateQuery);

	  // Then, we will print a header for each State in which offices are located:	  
	  while ($rows2 = mysql_fetch_array($stateResult2)) {
	    echo "<h2>Offices in the State of <a id=\"" . $rows2['state'] . "\"></a>" .
	    $rows2['state'] . ":</h2><br />"; 
	    reset($rows3);

	    // And we then pull a list of offices which fall within that state,
	    // separated into three columns.
	    $i = 0;
	    echo "<table style=\"width:670px\"><tr>";
	    while ($rows3 = mysql_fetch_array($officeResult)) {
	      if ($rows2['state'] == $rows3['office_state']) {
		if ($i % 3 == 0) { echo "</tr><tr>"; }
		echo "<td width=\"200\"><b>" . $rows3['street_address'] .
		"<br />" . $rows3['citystatezip'] . "<br /><a href=\"http://www.google.com/maps/dir/current+position/" .
		urlencode($rows3['street_address'] . " " . $rows3['citystatezip']) .
		"\" target=\"_blank\">Click for map</a><br /><hr /><br /><br /></td>";
		$i++;
              }
	    }
	    echo "</tr></table>";
	    mysql_data_seek($officeResult, 0);       
	  }
	}
	 
	else echo "An error occurred: " . mysql_error();

        
?>

</body>

</html>

