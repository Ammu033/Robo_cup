var showExhibitTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/interface/showexhibit",
    messageType: "std_msgs/String"
});


showExhibitTopic.subscribe(function(msg) {
    console.log('listener interface show exhibit msg.data='+msg.data);
    Show_exhibit(msg.data);
});

function Show_exhibit(msg) {
    // find the exhibit content
    var json_filepath = '';
    if (msg != '') {
    json_filepath = "config/exhibitors_definition.json";
    // load the content of the page
    $.getJSON(json_filepath, function(data) {
        $("[role=content]").load("exhibitorPage.html", function(){
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
  }