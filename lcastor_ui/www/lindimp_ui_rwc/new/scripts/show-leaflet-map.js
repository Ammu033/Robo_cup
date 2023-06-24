var mapMarkerClickedTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/lindimp_ui_rwc/map_marker_clicked",
    messageType: "std_msgs/String"
});

var mapMarkerClickedTopicString = new ROSLIB.Message({
    data: ""
});

var markers = [];

function Show_leaflet_map() {
    $("[role=content]").load("display-map-leaflet.html", function() {
        $.getJSON("config/exhibitors_definition.json", function(data) {
            var center = [0, 0];
            var map = L.map('map', {
            center: center,
            zoom: 1,
            minZoom: 1,
            maxZoom: 1,
            crs: L.CRS.Simple,
            attributionControl: false
            });

            // dimentions of the image
            var w = 746,
                h = 500;

            var southWest = map.unproject([0, h], map.getZoom());
            var northEast = map.unproject([w, 0], map.getZoom());
            var imageBounds = new L.LatLngBounds(southWest, northEast);
            var overlay = L.imageOverlay("images/collection-map.jpg", imageBounds, {interactive: true}).addTo(map);

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
                    '<rwc-button-action-start data-id="describeExhibitButton" data-class="describeExhibitButton" data-busy-class="describeExhibitButtonBusy" data-action="describeExhibit" data-action-parameters="' + key + '" data-text="Tell me about"></rwc-button-action-start>' +
                    '<rwc-button-action-start data-id="goToExhibitButton" data-class="goToExhibitButton" data-busy-class="goToExhibitButtonBusy" data-action="goToAndDescribeExhibit" data-action-parameters="' + key + '" data-text="Go there"></rwc-button-action-start>' +
                    '</div>'
                    : 'Click on a marker!');
            };
            info.addTo(map);

            var iconSize = [40, 60];

            var idIcon = L.Icon.extend({
                options: {
                    id: "",
                    iconUrl: 'images/marker-icon-2x-blue.png',
                    iconSize: iconSize,
                    iconAnchor: [12, 41],
                    popupAnchor: [1, -34],
                    shadowSize: [41, 41]
                }
            });

            var icons = [new idIcon({
                iconUrl: 'images/marker-icon-2x-blue.png',
                iconSize: iconSize,
                iconAnchor: [12, 41],
                popupAnchor: [1, -34],
                shadowSize: [41, 41]
            }),
            new idIcon({
                iconUrl: 'images/marker-icon-2x-green.png',
                iconSize: iconSize,
                iconAnchor: [12, 41],
                popupAnchor: [1, -34],
                shadowSize: [41, 41]
            }),
            new idIcon({
                iconUrl: 'images/marker-icon-2x-red.png',
                iconSize: iconSize,
                iconAnchor: [12, 41],
                popupAnchor: [1, -34],
                shadowSize: [41, 41]
            }),
            new idIcon({
                iconUrl: 'images/marker-icon-2x-yellow.png',
                iconSize: iconSize,
                iconAnchor: [12, 41],
                popupAnchor: [1, -34],
                shadowSize: [41, 41]
            })];

            markers = [];
            for (var t=0; t<data["tours"].length; t++) {
                for (var e=0; e<data["tours"][t]["exhibitors"].length; e++) {
                    for (i=0; i<data["exhibitors"].length; i++) {
                        if (data["tours"][t]["exhibitors"][e] == data["exhibitors"][i]["key"]) {
                            var x = data["exhibitors"][i]["map_image_position"][0];
                            var y = data["exhibitors"][i]["map_image_position"][1];
                            markers[i] = L.marker(map.unproject([x, y], map.getZoom()),  {id: data["exhibitors"][i]["key"], icon: icons[t], interactive: true, riseOnHover: true}).addTo(map);

                            // popup
                            markers[i].on('click', (function() {
                                var xi = x;
                                var yi = y;
                                var offset = -(iconSize[1] * 0.7);
                                var title = data["exhibitors"][i]["title"];
                                var key = data["exhibitors"][i]["key"];
                                var mapi = map;
                                var image = data["exhibitors"][i]["tile_image"];
                                return (function(e) {
                                    if (typeof receiver !== "undefined"){
                                        this._icon.setAttribute("id", String(key));
                                        L.popup({maxWidth: 150, autopan: true})
                                        .setLatLng(mapi.unproject([xi * mapi.getZoom(), yi * mapi.getZoom() + offset]))
                                        .setContent( key + ": <b>"+title +"</b>")
                                        .openOn(mapi);
                                        info.update(title, image, key);
                                    } else {
                                        L.popup({maxWidth: 150, autopan: true})
                                        .setLatLng(mapi.unproject([xi * mapi.getZoom(), yi * mapi.getZoom() + offset]))
                                        .setContent( key + ": <b>"+title +"</b>")
                                        .openOn(mapi);
                                        info.update(title, image, key);
                                        mapMarkerClickedTopicString.data = String(key);
                                        mapMarkerClickedTopic.publish(mapMarkerClickedTopicString);
                                    }
                                    console.log(this);
                                });
                            }) ());
                        }
                    }
                }
            }
        });
    });
}