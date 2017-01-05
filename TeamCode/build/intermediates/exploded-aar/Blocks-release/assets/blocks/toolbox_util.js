/**
 * @fileoverview Toolbox utilities.
 * @author lizlooney@google.com (Liz Looney)
 */

/**
 * Fetches the toolbox (as xml) and calls the callback.
 */
function fetchToolbox(callback) {
  if (typeof blocksIO !== 'undefined') {
    // FtcBlocks.html is within the WebView component within the Android app.
    fetchToolboxViaBlocksIO(callback);
  } else if (window.location.protocol === 'http:') {
    // FtcBlocks.html is in a browser, loaded as an http:// URL.
    fetchToolboxViaHttp(callback);
  } else if (window.location.protocol === 'file:') {
    // FtcBlocks.html is in a browser, loaded as a file:// URL.
    fetchToolboxViaFile(callback);
  }
}

function addToolboxIcons(workspace) {
  addToolboxIconsForChildren(workspace.toolbox_.tree_.getChildren());
}

function addToolboxIconsForChildren(children) {
  for (var i = 0, child; child = children[i]; i++) {
    if (child.getChildCount() > 0) {
      addToolboxIconsForChildren(child.getChildren());
    } else {
      child.setIconClass('toolbox-node-icon ' + child.getText() + '-icon');
    }
  }
}

//..........................................................................
// Code used when FtcBlocks.html is within the WebView component within the
// Android app.

function fetchToolboxViaBlocksIO(callback) {
  var xmlToolbox = blocksIO.fetchToolbox();
  if (xmlToolbox) {
    callback(xmlToolbox, '');
  } else {
    callback(null, 'Fetch toolbox failed.');
  }
}

//..........................................................................
// Code used when FtcBlocks.html is in a browser, loaded as a http:// URL.

function fetchToolboxViaHttp(callback) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', '/toolbox', true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        var xmlToolbox = xhr.responseText;
        callback(xmlToolbox, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(null, 'Fetch toolbox failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send();
}

//..........................................................................
// Code used when FtcBlocks.html is in a browser, loaded as a file:// URL.

function fetchToolboxViaFile(callback) {
  var xmlToolbox =
      '<xml id="toolbox" style="display: none">' +
      '<category name="LinearOpMode">' +
      '<block type="linearOpMode_waitForStart"></block>' +
      '<block type="linearOpMode_idle"></block>' +
      '<block type="linearOpMode_sleep">' +
      '<value name="MILLISECONDS">' +
      '<block type="math_number">' +
      '<field name="NUM">1000</field>' +
      '</block>' +
      '</value>' +
      '</block>' +
      '<block type="linearOpMode_opModeIsActive"></block>' +
      '<block type="linearOpMode_isStarted"></block>' +
      '<block type="linearOpMode_isStopRequested"></block>' +
      '</category>' +
      '<category name="Miscellaneous" colour="200">' +
      '<block type="comment">' +
      '<field name="COMMENT">Enter your comment here!</field>' +
      '</block>' +
      '</category>' +
      '</xml>';
  callback(xmlToolbox, '');
}
