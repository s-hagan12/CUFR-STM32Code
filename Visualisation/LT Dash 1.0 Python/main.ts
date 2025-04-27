//TO COMPILE THIS, RUN 'tsc'.
const express = require('express');
const app = express();
const ip = require('ip');
const ipAddress = ip.address();
const fs = require('fs/promises');
import { error } from 'console';

const PORT_TO_WEBSERVER = 4000;
const PORT_TO_WEBSOCKET = 4040;

//---------------
//SERVING WEBSERVER:
//---------------
app.use(express.json());
app.use(express.static('./public'));

app.listen(PORT_TO_WEBSERVER, () => {
    console.log(`Express WebServer running on ${ipAddress}:${PORT_TO_WEBSERVER}`);
    console.log('Express WebServer listening on port ' + PORT_TO_WEBSERVER);
});