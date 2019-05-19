import * as React from "react";
import * as ReactDOM from "react-dom";
import { Router } from "react-router-dom";
import { Provider } from "react-redux";
import configureStore from "./store";
import App from "./App";
// tslint:disable-next-line
import { hot } from "react-hot-loader";
import { history } from "./utils/custom-history";
import "noty/lib/noty.css";
import "noty/lib/themes/nest.css";

(window as any).store = configureStore.store;
const HotApp = hot(module)(App);

const rootEl = document.getElementById("root");
ReactDOM.render(
  <Provider store={configureStore.store}>
    <Router history={history}>
      <HotApp />
    </Router>
  </Provider>,
  rootEl
);
