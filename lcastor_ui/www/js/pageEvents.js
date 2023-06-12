// // load JS function
// loadJS = function(href) {
//   var jsLink = $("<script type='text/javascript' src='"+href+"'>");
//   $("body").append(jsLink);
// };
//
// // load CSS function
// loadCSS = function(href) {
//   var cssLink = $("<link rel='stylesheet' type='text/css' href='"+href+"'>");
//   $("head").append(cssLink);
// };

// do not cache the json file
$.ajaxSetup({ cache: true });

// set event whenever the page is refreshed, send ros message
window.onbeforeunload = function(){
    Signal_refreshPage();
};

window.onmousemove = function() {
    Signal_mousemoved();
    //console.log("The mouse moved!");
}

function shuffle(array) {
  for (let i = array.length - 1; i > 0; i--) {
    let j = Math.floor(Math.random() * (i + 1)); // random index from 0 to i
    [array[i], array[j]] = [array[j], array[i]]; // swap elements
  }
}

function Show_mant() {
  $.when(
    // load header builder
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();
    $("[role=content]").load("/maintenancePage.html", function(){

    });
  });
}

function Show_waitCharging() {
  $.when(
    // load header builder
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();
    $("[role=content]").load("/waitChargingPage.html", function(){

    });
  });
}

function Show_waitRelease() {
  $.when(
    // load header builder
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();
    $("[role=content]").load("/waitReleasePage.html", function(){

    });
  });
}

function Show_home() {
  $.when(
    // load header builder
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();

    // if the navigation is disabled stay only on the map page
    Check_exhist_param("/navigation_enabled").then((params) => Check_param_value(params).then(value => {
      if (!value) {
        Show_leaflet_map();
      }
    }));

    // load the content of the page
    $("[role=content]").load("home-page.html", function() {

      var name_file='/services.json';

      $.getJSON(name_file, function(data) {
        // add the cards in the deck
          var data_services=data.services;
          for (var i=0;i<data_services.length;i++) {
              var item=data_services[i];
          $('.card-deck').append(
            $('<a>').attr("class","card border-dark m-3").attr("id", item.id).attr("href","#").attr("style", "box-shadow: 5px 5px 50px #555; border-width:2px").append(
                      $('<img>').attr("class","card-img-top").attr("src",item.image).attr("style", "height:100%; opacity: .8").attr("draggable", "false"),
              $('<div>').attr("class", "card-img-overlay text-dark").append(
                $('<h5>').attr("class","card-title").attr("style", "color: #FFC300; text-shadow: -1px -1px 0 #111, 1px -1px 0 #111, -1px 1px 0 #111, 1px 1px 0 #111; font-size:30px").text(item.title),
                $('<p>').attr("class","card-text").text(item.description)
              )
            )
              );
          // Now when we press each service button we send a topic message
          // console.log("registered" + item.id);
          $("#"+item.id).onclick( (function() {
            var btn_id = item.id; //copy the value in a new var
            return function() {

                  if (btn_id == "guided_tours_server") {
                    //Demand_task(btn_id, "", 60 * 30);
                    SafeButtonEvent(function(){
                        Show_available_tours();
                    }, btn_id);
                  } else if (btn_id == "map_page") {
                      SafeButtonEvent(function(){
                          Show_leaflet_map();
                      }, btn_id);
                  } else if (btn_id == "videos_page") {
                      SafeButtonEvent(function(){
                          Show_videos();
                      }, btn_id);
                  }
            };
          })() );
        }

      });
    });

  });
}

function Show_available_tours() {
  $.when(
    // $.getStylesheet("/css/ladda.min.css"),
    $.getStylesheet("/css/gridtab.css"),

    // load header builder
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    // load spin js
    // $.getScript("/js/spin.min.js"),
    // load ladda js
    // $.getScript("/js/ladda.min.js"),
    // load gridtab js
    $.getScript("/js/gridtab.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();

    // load the content of the page
    json_filepath = "/config/exhibitors_definition.json";

    // load the content of the page
    $.getJSON(json_filepath, function(data) {
      $("[role=content]").load("/services/guided_tours/display-tours.html", function() {
        $("[role=container-title]").text("Please, select one of the available tours.")
        var tours = data["tours"];

        //shuffle the tours array so that appears in randomized order in the page
        shuffle(tours);

        // let's get the elements we need
        $.get("/services/guided_tours/display-tours-tab.html", '', function(response, status, xhr) {
          var t_title = new DOMParser().parseFromString(response, "text/html").querySelector("[role=tour-title]");
          var t_content = new DOMParser().parseFromString(response, "text/html").querySelector("[role=tour-content]");
          var t_btn = new DOMParser().parseFromString(response, "text/html").querySelector("[role=tour-btn]");
          $.get("/services/guided_tours/display-tours-tile.html", '', function(response1, status1, xhr1) {
            var s_img = new DOMParser().parseFromString(response1, "text/html").querySelector("[role=stop-img]");
            var s_desc = new DOMParser().parseFromString(response1, "text/html").querySelector("[role=stop-desc]");

            for (var i=0; i<tours.length; i++) {
              t_title_c = t_title.cloneNode(true);
              t_title_c.innerHTML = tours[i]["name"];
              $("[role=tours-container]").append(t_title_c);

              t_content_c = t_content.cloneNode(true);

              var tour_stop_keys = tours[i]["exhibitors"];
              var exhibitors = data["exhibitors"];
              for (var k=0; k<tour_stop_keys.length; k++) {
                for (var e=0; e<exhibitors.length; e++) {
                  if (exhibitors[e]["key"] == tour_stop_keys[k]) {
                    s_img_c = s_img.cloneNode(true);
                    s_img_c.getElementsByTagName("img")[0].setAttribute("src", exhibitors[e]["tile_image"]);
                    s_desc_c = s_desc.cloneNode(true);
                    s_desc_c.innerHTML = exhibitors[e]["title"]
                    t_content_c.getElementsByTagName("dl")[0].append(s_img_c);
                    t_content_c.getElementsByTagName("dl")[0].append(s_desc_c);
                  }
                }
              }
              $("[role=tours-container]").append(t_content_c);
              // t_btn_c = t_btn.cloneNode(true);
              // t_btn_c.onmousedown
              t_content_c.querySelector("[role=tour-btn]").innerHTML = "Start this tour!"
              t_content_c.querySelector("[role=tour-btn]").onclick = (function() {
                var tour_id = tours[i]["key"]; //copy the value in a new var
                return function(){
                    SafePhysicalButtonEvent(function() {
                  // Signal_buttonPressed(btn_id);
                  // Check_exhist_param("/interface_enabled").then((params) => Check_param_value(params).then(value => {
                  //   if (value)
                        Start_tour_task(tour_id);
                  //   }
                  // }));
                }, tour_id);
                }
              }) ();
            }

            $('.gridtab-1').gridtab({
              grid: 6,
              tabPadding: 0,
              borderWidth: 4,
              contentPadding: 30,
              tabPadding: 15,
              // activeTabBackground: '#563C55',
              contentBorderColor: '#ffc107',
              tabBorderColor: '#ffc107',
              responsive: [{
                breakpoint: 991,
                settings: {
                  grid: 4,
                  contentPadding: 30
                }
              }, {
                breakpoint: 767,
                settings: {
                  grid: 3,
                  contentPadding: 20
                }
              }, {
                breakpoint: 520,
                settings: {
                  grid: 2
                }
              }],
              config:{
                layout: 'tab',
                activeTab:1,
                // scrollToTab:true,
                keepOpen: true,
              }
            });
            $('.gridtab-6').gridtab({
              grid: 4,
              contentPadding: 20,
              tabBorderColor: '#563C55',
              contentBorderColor: '#563C55',
              // activeTabBackground:'#563C55',
              config:{
                layout:'tab',
                activeTab:1,
                showClose:false,
                showArrows:false,
                scrollToTab:false,
                keepOpen: true
              }
            });

          }, 'html');
        }, 'html');
      });
    });
  });
}

function Show_exhibit(msg) {
  $.when(
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();

    // find the exhibit content
    var json_filepath = '';
    if (msg != '') {
      json_filepath = "/config/exhibitors_definition.json";
      // load the content of the page
      $.getJSON(json_filepath, function(data) {
        $("[role=content]").load("/exhibitorPage.html", function(){
          var key = msg.split("_")[0]
          var title, desc, image;
          for (i=0; i<data["exhibitors"].length; i++) {
            if (data["exhibitors"][i]["key"] == key) {
              title = data["exhibitors"][i]["title"]
              console.log(msg.split("_"))
              if (msg.split("_").length > 1 && msg.split("_")[1] == "additional") {
                desc = data["exhibitors"][i]["additional_description"]
                image = data["exhibitors"][i]["additional_image"]
              } else {
                desc = data["exhibitors"][i]["description"]
                image = data["exhibitors"][i]["image"]
              }
            }
          }
          $("[role=title]").append(title);
          $("[role=description]").append(desc);
          $("[role=image]").attr("src", image);
          console.log("Showing exhibit" + title)
        // and fill it with content
        });
      });
    } else {
      //ideally we should signal that the exhibit doesn't exhist
      console.log("Exhibit " + msg + " not found!");
    }
  });
}

function Show_navigation(msg) {
  $.when(
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();

    // find the exhibit content
    var json_filepath = '';
    if (msg != '') {
      json_filepath = "/config/exhibitors_definition.json";
      // load the content of the page
      $.getJSON(json_filepath, function(data) {
        $("[role=content]").load("/navigation-page.html", function(){

          var key = msg.split("_")[0]
          var title, desc, image;
          for (i=0; i<data["exhibitors"].length; i++) {
            if (data["exhibitors"][i]["key"] == key) {
              title = data["exhibitors"][i]["title"]
              image = data["exhibitors"][i]["lead_image"]
            }
          }

          $("[role=title]").append("<i>Going to:</i> <b>" + title + "</b>");
          // $("[role=description]").append(desc);
          $("[role=image]").attr("src", image);

          console.log("Showing exhibit" + title)
        // and fill it with content
        });
      });

    } else {
      //ideally we should signal that the exhibit doesn't exhist
      console.log("Exhibit " + msg + " not found!");
    }
  });
}

function Show_nreal(msg) {
  $.when(
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();

      // load the content of the page
      $.getJSON(json_filepath, function(data) {
        $("[role=content]").load("/nreal-page.html", function(){

          console.log("Showing nreal" + title)
        // and fill it with content
        });
      });

  });
}

function Show_leaflet_map() {
  $.when(
    $.getStylesheet("/css/leaflet.css"),
    $.getScript("/js/headerBuilder.js"),
    $.getScript("/js/footerBuilder.js"),
    $.getScript("/js/leaflet.js"),
    $.Deferred(function( deferred ){
      $( deferred.resolve );
    })
  ).done(function(){
    Signal_pageChange();

    $("[role=content]").load("/display-map-leaflet.html", function() {
      // set height to 100% to map div and all its anchestors
      // $("html").attr("style", "height: 100%");
      // $("body").attr("style", "height: 100%");
      // $("[role=content]").attr("style", "height: 100%");
      $.getJSON("/config/exhibitors_definition.json", function(data) {

        var center = [0, 0];
        var map = L.map('map', {
          center: center,
          zoom: 1,
          minZoom: 1,
          maxZoom: 1,
          crs: L.CRS.Simple,
          attributionControl: false,
          zoomControl: false
        });

        // dimentions of the image
        var w = 746,
            h = 500;

        var southWest = map.unproject([0, h], map.getZoom());
        var northEast = map.unproject([w, 0], map.getZoom());
        var imageBounds = new L.LatLngBounds(southWest, northEast);
        var overlay = L.imageOverlay("/images/collection-map.jpg", imageBounds, {interactive: true}).addTo(map);

        southWest = map.unproject([0, h], map.getZoom());
        northEast = map.unproject([w + 200, 60], map.getZoom());
        var mapBounds = new L.LatLngBounds(southWest, northEast);
        map.setMaxBounds(mapBounds);

        map.setView(map.unproject([w/2 + 100, h/2], map.getZoom()))

        map.dragging.disable();

        var info = L.control();
        info.onAdd = function (map) {
            this._div = L.DomUtil.create('div', 'info'); // create a div with a class "info"
            this.update();
            return this._div;
        };
        // method that we will use to update the control based on feature properties passed
        info.update = function (title, image, key) {
          this._div.innerHTML = '<h4>Exhibitor description</h4>' +  (title ?
              '<h4 style="text-align:center;color:black">' + title + '</h4><br />'+"<img src='" + image + "' draggable='false' style='width:100%;margin-left:auto;margin-right:auto;display: block;'>" +
              '<div style="vertical-align: middle">' +
              '<button style="display:block;margin: 5px auto 5px auto; width: 80%;" role="map-exhibit-tell" type=button class="btn btn-lg btn-info">Tell me about</button>' +
              '<button style="display:block;margin: 5px auto 5px auto; width: 80%;" role="map-exhibit-go" type=button class="btn btn-lg btn-success">Go there</button>' +
              '</div>'
              : 'Click on a marker!');
          $("[role=map-exhibit-tell]").click(function() {
            SafeButtonEvent(function(){
                Start_describe_task(key)
            }, key);
          });
          $("[role=map-exhibit-go]").click(function() {
              SafePhysicalButtonEvent(function(){
                  Start_gotoAndDescribe_task(key)
              }, key);
          });
        };
        info.addTo(map);

        var iconSize = [40, 60];
        var icons = [new L.Icon({
            iconUrl: '/images/marker-icon-2x-blue.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: iconSize,
            iconAnchor: [12, 41],
            popupAnchor: [1, -34],
            shadowSize: [41, 41]
          }),
          new L.Icon({
            iconUrl: '/images/marker-icon-2x-green.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: iconSize,
            iconAnchor: [12, 41],
            popupAnchor: [1, -34],
            shadowSize: [41, 41]
          }),
          new L.Icon({
            iconUrl: '/images/marker-icon-2x-red.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: iconSize,
            iconAnchor: [12, 41],
            popupAnchor: [1, -34],
            shadowSize: [41, 41]
          }),
          new L.Icon({
            iconUrl: '/images/marker-icon-2x-yellow.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: iconSize,
            iconAnchor: [12, 41],
            popupAnchor: [1, -34],
            shadowSize: [41, 41]
          })];

        // var tmarker = L.marker(map.unproject([100, 100], map.getMaxZoom()), {icon: greenIcon}).addTo(map);
        // tmarker.on('mouseover', function() {
        //     console.log("HOVER")
        //     alert("aaaaaaaaa");
        // });
        var markers = []
        for (var t=0; t<data["tours"].length; t++) {
          for (var e=0; e<data["tours"][t]["exhibitors"].length; e++) {
            for (i=0; i<data["exhibitors"].length; i++) {
              if (data["tours"][t]["exhibitors"][e] == data["exhibitors"][i]["key"]) {
                var x = data["exhibitors"][i]["map_image_position"][0];
                var y = data["exhibitors"][i]["map_image_position"][1];
                markers[i] = L.marker(map.unproject([x, y], map.getZoom()),  {icon: icons[t], interactive: true, riseOnHover: true}).addTo(map);
                // tooltip
                // markers[i].bindTooltip("my tooltip text");
                // markers[i].on("mouseover", function() { alert("diocane");})
                // markers[i].on("click", function() { alert("diocane1");})
                // markers[i].on("mousedown", function() { alert("diocane2");})

                // popup
                markers[i].on('mouseover', (function() {
                 var xi = x;
                 var yi = y;
                 var offset = -(iconSize[1] * 0.7);
                 var title = data["exhibitors"][i]["title"];
                 var key = data["exhibitors"][i]["key"];
                 var mapi = map;
                 var image = data["exhibitors"][i]["tile_image"];
                 return (function(e) {
                   L.popup({maxWidth: 150, autopan: true})
                     .setLatLng(mapi.unproject([xi * mapi.getZoom(), yi * mapi.getZoom() + offset]))
                     .setContent( key + ": <b>"+title +"</b>")
                     .openOn(mapi);
                   info.update(title, image, key);
                 });
                }) ());
                // markers[i].on('mouseout', function() {info.update();});
              }
            }
          }
        }
      });


    });
  });
}


function Remove_footer() {
  $.getScript("/js/footerDestroyer.js");
}
