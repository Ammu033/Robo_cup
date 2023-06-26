$(document).ready(function(){
    currentPageTopic.subscribe(function(msg){
        var currentPage = msg.data;
        if (currentPage === "/lindimp_ui_rwc/new/tours-page.html"){
            console.log("Tours page loaded!");
            Show_available_tours();
        } else if (currentPage === "/lindimp_ui_rwc/new/map-page.html"){
            console.log("Map page loaded!");
            $(document).ready(function(){
                setTimeout(function(){
                    Show_leaflet_map();
                    mapMarkerClickedTopic.subscribe(function(msg){
                        var clickedMarkerKey = msg.data;
                        markers.forEach(function(marker){
                            var id = marker.options.id;
                            if(id == clickedMarkerKey){$(marker._icon).trigger("click");}
                        });
                    });
                }, 500);
            });
        }
    });

    tourTitleClickedTopic.subscribe(function(msg){
        $("#" + msg.data).trigger("click");
    });

    exhibitCellClickedTopic.subscribe(function(msg){
        var exhibitCellID = msg.data;
        console.log(exhibitCellID);
        $(document.getElementById(exhibitCellID)).trigger("click");
    });
});