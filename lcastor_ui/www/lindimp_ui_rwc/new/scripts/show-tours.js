if (typeof receiver !== "undefined"){
  console.log("Receiver: " + receiver);
}

// ROS topics to track gribtab clicks
var tourTitleClickedTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/lindimp_ui_rwc/tourTitleClicked",
  messageType: "std_msgs/String"
});

var tourTitleClickedTopicString = new ROSLIB.Message({data: ""});

var exhibitCellClickedTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/lindimp_ui_rwc/exhibitCellClicked",
  messageType: "std_msgs/String"
});

var exhibitCellClickedTopicString = new ROSLIB.Message({data: ""});

// if (typeof receiver === "undefined"){
// ROS topic to track tours arrays
var toursArrayTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/lindimp_ui_rwc/toursArray",
  messageType: "std_msgs/String",
  latch: true
});

var toursArrayTopicString = new ROSLIB.Message({data: ""});
var tours;
var exhibitorsData;

// Shuffle function to randomise tour order
function shuffle(array) {
    for (let i = array.length - 1; i > 0; i--) {
      let j = Math.floor(Math.random() * (i + 1)); // random index from 0 to i
      [array[i], array[j]] = [array[j], array[i]]; // swap elements
    }
}


function Show_available_tours() {
    $.when(
      $.Deferred(function( deferred ){
        $( deferred.resolve );
      })
    ).done(function(){
      // load the content of the page
      json_filepath = "exhibitors_definition.json";
  
      // load the content of the page
      $.getJSON(json_filepath, function(data) {
        exhibitorsData = data;
        $("[role=content]").load("display-tours.html", function() {
          $("[role=container-title]").text("Please, select one of the available tours.")

          if (typeof receiver === "undefined"){
            tours = data["tours"];
            //shuffle the tours array so that appears in randomized order in the page
            shuffle(tours);
            toursArrayTopicString.data = JSON.stringify(tours);
            toursArrayTopic.publish(toursArrayTopicString);
            loadToursGridtab();
          } else {
            toursArrayTopic.subscribe(function(msg){
              tours = JSON.parse(msg.data);
              loadToursGridtab();
            });
          }
        });
      });
    });
  }

  function loadToursGridtab(){
    // let's get the elements we need
    $.get("display-tours-tab.html", '', function(response, status, xhr) {
      var t_title = new DOMParser().parseFromString(response, "text/html").querySelector("[role=tour-title]");
      var t_content = new DOMParser().parseFromString(response, "text/html").querySelector("[role=tour-content]");
      var t_btn = new DOMParser().parseFromString(response, "text/html").querySelector("[role=tour-btn]");
      $.get("display-tours-tile.html", '', function(response1, status1, xhr1) {
        var s_img = new DOMParser().parseFromString(response1, "text/html").querySelector("[role=stop-img]");
        var s_desc = new DOMParser().parseFromString(response1, "text/html").querySelector("[role=stop-desc]");

        for (var i=0; i<tours.length; i++) {
          var t_title_c = t_title.cloneNode(true);
          var tourKey = tours[i]["key"];
          t_title_c.setAttribute("id", "tour-" + tourKey);
          if (typeof receiver === "undefined"){
            $(t_title_c).on("click", function(){
              tourKey = this.id;
              tourTitleClickedTopicString.data = tourKey;
              tourTitleClickedTopic.publish(tourTitleClickedTopicString);
            });
          }
          t_title_c.innerHTML = tours[i]["name"];
          $("[role=tours-container]").append(t_title_c);

          t_content_c = t_content.cloneNode(true);

          var tour_stop_keys = tours[i]["exhibitors"];
          var exhibitors = exhibitorsData["exhibitors"];
          for (var k=0; k<tour_stop_keys.length; k++) {
            for (var e=0; e<exhibitors.length; e++) {
              if (exhibitors[e]["key"] == tour_stop_keys[k]) {
                s_img_c = s_img.cloneNode(true);
                s_img_c.setAttribute("id", "exhibit-" + exhibitors[e]["key"]);
                if (typeof receiver === "undefined"){
                  $(s_img_c).on("click", function(){
                    var exhibitKey = this.id;
                    exhibitCellClickedTopicString.data = exhibitKey;
                    exhibitCellClickedTopic.publish(exhibitCellClickedTopicString);
                  });
                }
                s_img_c.getElementsByTagName("img")[0].setAttribute("src", exhibitors[e]["tile_image"]);
                s_desc_c = s_desc.cloneNode(true);
                s_desc_c.innerHTML = exhibitors[e]["title"]
                t_content_c.getElementsByTagName("dl")[0].append(s_img_c);
                t_content_c.getElementsByTagName("dl")[0].append(s_desc_c);
              }
            }
          }
          $("[role=tours-container]").append(t_content_c);
          var startTourButton = t_content_c.querySelector("[role=tour-btn]");
          var tour_id = tours[i]["key"]; //copy the value in a new var
          startTourButton.setAttribute("data-action-parameters", tour_id);
        }

        $('.gridtab-1').gridtab({
          grid: 6,
          tabPadding: 0,
          borderWidth: 4,
          contentPadding: 30,
          tabPadding: 15,
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
  }