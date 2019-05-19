import * as React from "react";
import CONFIG from "../../../../config";
import configureStore from "../../../../store";
import "./index.scss";
import { isNil, map, has, clone, values, size, sortBy } from "lodash";
import * as hash from "object-hash";
import {
  Table,
  Column,
  AutoSizer,
  CellMeasurer,
  CellMeasurerCache
} from "react-virtualized";
import { IInitialState } from "../../../../reducers/initial-state";

const { rosoutRenderRateInMs, rosoutStaleDataCheckRateInMs } = CONFIG;
const { store } = configureStore;

interface IRosoutDetailsProps {
  robotName: string | undefined;
}

interface IRosoutDetailsState {
  messagesCache: {
    [hash: string]: {
      lastUpdate: number;
      count: number;
      obj: IInitialState["rosout"][string][number];
    };
  };
}

class RosoutDetails extends React.PureComponent<
  IRosoutDetailsProps,
  IRosoutDetailsState
> {
  private setTimeoutId: any;
  private messageCache = new CellMeasurerCache({
    fixedWidth: true,
    minHeight: 25
  });
  private fileNameCache = new CellMeasurerCache({
    fixedWidth: true,
    minHeight: 25
  });
  constructor(props: IRosoutDetailsProps) {
    super(props);
    this.state = {
      messagesCache: {}
    };
  }

  public componentDidMount() {
    this.updateMessagesCache();
  }

  public componentWillUnmount() {
    clearInterval(this.setTimeoutId);
  }

  private updateMessagesCache = () => {
    const { robotName } = this.props;
    const { messagesCache } = this.state;
    if (!isNil(store) && !isNil(robotName)) {
      const rosout = (store.getState() as IInitialState).rosout;
      const rosoutForRobot = rosout[robotName];
      map(rosoutForRobot, obj => {
        const objHash = hash(obj);
        if (has(messagesCache, objHash)) {
          messagesCache[objHash].count++;
          messagesCache[objHash].lastUpdate = -1 * performance.now();
        } else {
          const temp = clone(messagesCache);
          temp[objHash] = {
            // TODO: This should be handled at the time of message instead of render time. BUG.
            lastUpdate: -1 * performance.now(),
            count: 1,
            obj
          };
          this.setState({ messagesCache: temp });
        }
      });
    }
    this.setTimeoutId = setTimeout(
      this.updateMessagesCache,
      rosoutRenderRateInMs
    );
  };

  private rowGetter = ({ index }: { index: number }) => {
    const { messagesCache } = this.state;
    const messages = values(messagesCache) as Array<
      IRosoutDetailsState["messagesCache"][string]
    >;
    const sortedMessages = sortBy(messages, x => x.lastUpdate);
    return { ...sortedMessages[index].obj, count: sortedMessages[index].count };
  };

  public render() {
    const { messagesCache } = this.state;
    return (
      <React.Fragment>
        <div className="rosout">
          <AutoSizer disableHeight>
            {({ width }) => (
              <Table
                ref="Table"
                headerClassName="SAMPLECLASS"
                headerHeight={25}
                noRowsRenderer={() => <div>No rows</div>}
                overscanRowCount={10}
                rowClassName=""
                rowHeight={100}
                rowGetter={this.rowGetter}
                rowCount={size(messagesCache)}
                width={width}
                height={500}
              >
                <Column
                  label="Count"
                  dataKey="count"
                  disableSort={true}
                  width={width / 10}
                />
                <Column
                  label="Level"
                  dataKey="level"
                  disableSort={false}
                  width={width / 10}
                />
                <Column
                  label="NodeName"
                  dataKey="nodeName"
                  disableSort={false}
                  width={width / 5}
                />
                <Column
                  label="File"
                  dataKey="file"
                  disableSort={false}
                  width={width * (3 / 10)}
                  cellRenderer={args => (
                    <CellMeasurer
                      cache={this.fileNameCache}
                      columnIndex={0}
                      key={args.dataKey}
                      parent={args.parent}
                      rowIndex={args.rowIndex}
                    >
                      <div
                        className=""
                        style={{
                          whiteSpace: "normal"
                        }}
                      >
                        {args.rowData.file}
                      </div>
                    </CellMeasurer>
                  )}
                />
                <Column
                  label="Message"
                  dataKey="message"
                  disableSort={false}
                  width={width * (3 / 10)}
                  cellRenderer={args => (
                    <CellMeasurer
                      cache={this.messageCache}
                      columnIndex={0}
                      key={args.dataKey}
                      parent={args.parent}
                      rowIndex={args.rowIndex}
                    >
                      <div
                        className=""
                        style={{
                          whiteSpace: "normal"
                        }}
                      >
                        {args.rowData.message}
                      </div>
                    </CellMeasurer>
                  )}
                />
              </Table>
            )}
          </AutoSizer>
        </div>
      </React.Fragment>
    );
  }
}

export default RosoutDetails;
