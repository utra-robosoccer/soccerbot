export interface IInitialState {
  robots: Array<
    Partial<{
      hostname: string;
      previousConfigString: string | undefined;
      currentConfigString: string | undefined;
      name: string;
      status: "online" | "offline";
      launchGroups: Array<{
        name: string;
        active: boolean;
        launchConfigs: Array<{
          name: string;
          ticked: boolean;
          status: string;
          statename: string;
        }>;
      }>;
    }>
  >;
  robotsNotifications: Array<{
    hash: number;

    robotName: string;
    heading: string;
    content: string;

    type: "CONFIG_NOT_FOUND";
    level: "error" | "warning" | "info";
  }>;
  rosmon: {
    [robotName: string]: {
      [launchGroup: string]: {
        [launchConfig: string]: {
          [node: string]: {
            state: number;
            lastUpdateTimeStamp: number;
            restartCount: number;
            stats: {
              cpu: number;
              memory: number;
            };
          };
        };
      };
    };
  };
  cmdGlobal: number;
  rosout: {
    [robotName: string]: Array<{
      level: string;
      nodeName: string;
      message: string;
      file: string;
    }>;
  };
}

const initialState: IInitialState = {
  robots: [],
  robotsNotifications: [],
  rosmon: {},
  cmdGlobal: 1,
  rosout: {}
};

export default initialState;
