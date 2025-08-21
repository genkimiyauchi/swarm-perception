# argos3-webclient
A webclient for ARGoS using https://github.com/NESTLab/argos3-webviz



# Install nlohmann-json3-dev

- Enable the repository:
```bash
sudo add-apt-repository ppa:team-xbmc/ppa
```

- Update the package index:
```bash
sudo apt-get update
```

- Install nlohmann-json3-dev deb package:
```bash
sudo apt-get install nlohmann-json3-dev
```

Reference: https://ubuntu.pkgs.org/18.04/kodi-arm64/nlohmann-json3-dev_3.1.2-2~bionic_all.deb.html



# How to create websocket-as-promised js file

- Install parcel js
```bash
npm i websocket-as-promised
```
- Write a javascript file named `websocket-as-promised.js` with content
```javascript
window.WebSocketAsPromised = require('websocket-as-promised');
```
- run
```bash
parcel build websocket-as-promised.js
```

