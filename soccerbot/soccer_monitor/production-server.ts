/* tslint:disable */
import * as Webpack from "webpack";
import WebpackConfig from "./webpack.config.prod";
import { isNil } from "lodash";
import * as path from "path";
import { Request, Response } from "express";
const express = require("express");
const compression = require("compression");

const UI_HOST = process.env.UI_HOST || "localhost";
const UI_PORT = !isNil(process.env.UI_PORT)
  ? Number.parseInt(process.env.UI_PORT)
  : 9099;
const ROSBRIDGE_URL = process.env.ROSBRIDGE_URL || "ws://localhost:9091";

WebpackConfig.plugins!.push(
  new Webpack.DefinePlugin({
    ROSBRIDGE_URL: JSON.stringify(ROSBRIDGE_URL)
  })
);

const app = express();
app.use(compression());
app.use(express.static(`${__dirname}/dist`));
app.get("*", function(req: Request, res: Response) {
  res.sendFile(path.resolve(`${__dirname}/dist`, "index.html"));
});

const compiler = Webpack(WebpackConfig);
compiler.run(() => {
  app.listen(UI_PORT, UI_HOST, () =>
    console.log(`UI server started on ${UI_HOST}:${UI_PORT}`)
  );
});
