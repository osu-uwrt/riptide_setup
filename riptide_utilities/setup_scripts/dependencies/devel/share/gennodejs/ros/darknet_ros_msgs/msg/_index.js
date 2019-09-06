
"use strict";

let BoundingBox = require('./BoundingBox.js');
let BoundingBoxes = require('./BoundingBoxes.js');
let CheckForObjectsAction = require('./CheckForObjectsAction.js');
let CheckForObjectsGoal = require('./CheckForObjectsGoal.js');
let CheckForObjectsActionResult = require('./CheckForObjectsActionResult.js');
let CheckForObjectsResult = require('./CheckForObjectsResult.js');
let CheckForObjectsFeedback = require('./CheckForObjectsFeedback.js');
let CheckForObjectsActionFeedback = require('./CheckForObjectsActionFeedback.js');
let CheckForObjectsActionGoal = require('./CheckForObjectsActionGoal.js');

module.exports = {
  BoundingBox: BoundingBox,
  BoundingBoxes: BoundingBoxes,
  CheckForObjectsAction: CheckForObjectsAction,
  CheckForObjectsGoal: CheckForObjectsGoal,
  CheckForObjectsActionResult: CheckForObjectsActionResult,
  CheckForObjectsResult: CheckForObjectsResult,
  CheckForObjectsFeedback: CheckForObjectsFeedback,
  CheckForObjectsActionFeedback: CheckForObjectsActionFeedback,
  CheckForObjectsActionGoal: CheckForObjectsActionGoal,
};
