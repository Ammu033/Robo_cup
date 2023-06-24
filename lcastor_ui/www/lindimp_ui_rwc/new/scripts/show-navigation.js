var showNavigationTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/interface/shownavigation",
    messageType: "std_msgs/String"
});


showNavigationTopic.subscribe(function(msg) {
    console.log('listener interface show navigation msg.data='+msg.data);
    Show_navigation(msg.data);
});

function Show_navigation(msg) {
    // find the exhibit content
    var json_filepath = '';
    if (msg != '') {
    json_filepath = "config/exhibitors_definition.json";
    // load the content of the page
    $.getJSON(json_filepath, function(data) {
        $("[role=content]").load("navigation-page.html", function(){

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
}