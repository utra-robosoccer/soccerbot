/* tslint:disable */
import * as path from "path";
import * as HtmlWebpackPlugin from "html-webpack-plugin";
import * as webpack from "webpack";

const CleanWebpackPlugin = require("clean-webpack-plugin");

const config: webpack.Configuration = {
  entry: ["./src/index.tsx"],
  output: {
    filename: "bundle.[contenthash].js",
    path: path.join(__dirname, "dist")
  },

  mode: "production",
  devtool: "source-map",

  resolve: {
    extensions: [".ts", ".tsx", ".js", ".json"]
  },

  plugins: [
    new CleanWebpackPlugin(["dist"]),
    new HtmlWebpackPlugin({
      chunksSortMode: "dependency",
      template: path.resolve(__dirname, "./src/index.ejs"),
      title: "sootballs-monitor",
      production: true
    }),
    new webpack.HashedModuleIdsPlugin(),
    new webpack.DefinePlugin({
      "process.env.NODE_ENV": JSON.stringify("production")
    })
  ],

  optimization: {
    runtimeChunk: "single",
    splitChunks: {
      cacheGroups: {
        vendor: {
          test: /[\\/]node_modules[\\/]/,
          name: "vendors",
          chunks: "all"
        }
      }
    }
  },

  module: {
    rules: [
      {
        exclude: path.resolve(__dirname, "node_modules"),
        include: path.resolve(__dirname, "src"),
        test: /\.tsx?$/,
        use: [
          {
            loader: "babel-loader",
            options: {
              babelrc: true,
              plugins: ["react-hot-loader/babel", "lodash"]
            }
          },
          {
            loader: "awesome-typescript-loader",
            options: {
              configFileName: path.resolve(__dirname, "tsconfig.prod.json")
            }
          }
        ]
      },
      {
        enforce: "pre",
        loader: "source-map-loader",
        test: /\.js$/
      },
      {
        test: /\.(css|scss)$/,
        use: ["style-loader", "css-loader", "postcss-loader", "sass-loader"]
      },
      {
        test: /\.(gif|png|jpe?g|svg|woff|woff2|eot|ttf)$/i,
        use: ["url-loader?limit=10000&name=[path][name].[ext]"]
      },
      {
        test: /\.ya?ml$/,
        loader: "yaml-import-loader",
        options: {
          importRoot: false,
          importNested: true,
          importKeyword: "import",
          importRawKeyword: "import-raw",
          output: "object",
          parser: {
            types: [],
            schema: require("js-yaml").SAFE_SCHEMA,
            allowDuplicate: true,
            onWarning: undefined
          }
        }
      }
    ]
  }
};

export default config;
