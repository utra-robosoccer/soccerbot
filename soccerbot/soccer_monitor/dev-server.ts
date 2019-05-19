/* tslint:disable */
import * as Webpack from "webpack";
import * as WebpackDevServer from "webpack-dev-server";
import WebpackConfig from "./webpack.config.dev";
import { isNil } from "lodash";

const UI_HOST = process.env.UI_HOST || "localhost";
const UI_PORT = !isNil(process.env.UI_PORT)
  ? Number.parseInt(process.env.UI_PORT)
  : 9098;
const ROSBRIDGE_URL = process.env.ROSBRIDGE_URL || "ws://localhost:9091";

WebpackConfig.plugins!.push(
  new Webpack.DefinePlugin({
    ROSBRIDGE_URL: JSON.stringify(ROSBRIDGE_URL)
  })
);

const compiler = Webpack(WebpackConfig);
const devServerOptions = Object.assign({}, WebpackConfig.devServer);
const server = new WebpackDevServer(compiler, devServerOptions);

server.listen(UI_PORT, UI_HOST, () => {
  console.log(`Starting server on ${UI_HOST}:${UI_PORT}`);
});
