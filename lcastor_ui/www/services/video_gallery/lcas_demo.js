/*
 * blueimp Gallery Demo JS
 * https://github.com/blueimp/Gallery
 *
 * Copyright 2013, Sebastian Tschan
 * https://blueimp.net
 *
 * Licensed under the MIT license:
 * https://opensource.org/licenses/MIT
 */

/* global blueimp, $ */

$(function () {
  'use strict'

  
  var youTubeOptions = {
    // The list object property (or data attribute) with the YouTube video id:
    youTubeVideoIdProperty: 'youtube',
    // Optional object with parameters passed to the YouTube video player:
    // https://developers.google.com/youtube/player_parameters
    youTubePlayerVars: undefined,
    // Require a click on the native YouTube player for the initial playback:
    youTubeClickToPlay: false
};
  
  // Initialize the Gallery as video carousel:
  blueimp.Gallery([
      {
          title: 'HRI 2017 info-terminal',
          href: 'https://www.youtube.com/watch?v=2YEsql9_FjY',
          type: 'text/html',
          youtube: '2YEsql9_FjY',
          poster: 'https://img.youtube.com/vi/2YEsql9_FjY/maxresdefault.jpg'
      },
      {
          title: 'STRANDS movie',
          href: 'https://www.youtube.com/watch?v=67ul8Ix8ehs',
          type: 'text/html',
          youtube: '67ul8Ix8ehs',
          poster: 'https://img.youtube.com/vi/67ul8Ix8ehs/maxresdefault.jpg'
      },
      {
          title: 'Real-Time Multisensor People Tracking for Mobile Robots',
          href: 'https://www.youtube.com/watch?v=zdnvhQU1YNo',
          type: 'text/html',
          youtube: 'zdnvhQU1YNo',
          poster: 'https://img.youtube.com/vi/zdnvhQU1YNo/maxresdefault.jpg'
      },
      {
          title: 'Online Learning for Human Classification in 3D LiDAR-based Tracking',
          href: 'https://www.youtube.com/watch?v=bjztHV9rC-0',
          type: 'text/html',
          youtube: 'bjztHV9rC-0',
          poster: 'https://img.youtube.com/vi/bjztHV9rC-0/maxresdefault.jpg'
      },
      {
          title: 'ENRICHME robot demo',
          href: 'https://www.youtube.com/watch?v=98bVFkKoRl0',
          type: 'text/html',
          youtube: '98bVFkKoRl0',
          poster: 'https://img.youtube.com/vi/98bVFkKoRl0/maxresdefault.jpg'
      },
      {
          title: 'Persistent Localization and Life-long Mapping in Changing Environments using FreMEn',
          href: 'https://www.youtube.com/watch?v=8cNo2BquNOU',
          type: 'text/html',
          youtube: '8cNo2BquNOU',
          poster: 'https://img.youtube.com/vi/8cNo2BquNOU/maxresdefault.jpg'
      },
      {
          title: 'Thorvald in Action',
          href: 'https://www.youtube.com/watch?v=KGrmThshIYw',
          type: 'text/html',
          youtube: 'KGrmThshIYw',
          poster: 'https://img.youtube.com/vi/KGrmThshIYw/maxresdefault.jpg'
      },
      {
          title: "ZoidBot as Santa's little helper",
          href: 'https://www.youtube.com/watch?v=Oq1Axl45FCA',
          type: 'text/html',
          youtube: 'Oq1Axl45FCA',
          poster: 'https://img.youtube.com/vi/Oq1Axl45FCA/maxresdefault.jpg'
      }
    
    
    
  ], {
    container: '#blueimp-video-carousel',
    carousel: true
  })
})
