// Function that loads a css file
// always pass an absolute path
(function($) {
  $.getStylesheet = function (href) {
    var link_elements = $('link')
    href = window.location.protocol + "//" + window.location.host + href
    for (var i=0; i<link_elements.length; i++){
      if ( link_elements[i].href == href){
        return;
      }
    }
    var $d = $.Deferred();
    var $link = $('<link/>', {
       rel: 'stylesheet',
       type: 'text/css',
       href: href
    }).appendTo('head');
    $d.resolve($link);
    return $d.promise();
  };
})(jQuery);


// // Loads css and js files executing a callback afterwards
// function loadResources(css_list, js_list, callback) {
//   $.when(
//     function() {
//       // load css files
//       for (var i=0; i<css_list.length; i++) {
//         $.getStylesheet(css_list[i])
//       }
//       // load js files
//       for (var i=0; i<js_list.length; i++) {
//         $.getScript(js_list[i])
//       }
//     },
//     $.Deferred(function( deferred ){
//       $( deferred.resolve );
//     })
//   ).done(callback);
// }
