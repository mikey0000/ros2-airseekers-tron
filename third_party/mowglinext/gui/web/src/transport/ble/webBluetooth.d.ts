// Minimal Web Bluetooth ambient types (https://webbluetoothcg.github.io/web-bluetooth/) for the
// parts gattLink.ts uses. TypeScript's DOM lib does not ship them and the project avoids a
// dependency on @types/web-bluetooth.

type BluetoothServiceUUID = number | string;
type BluetoothCharacteristicUUID = number | string;

interface BluetoothLEScanFilter {
    name?: string;
    namePrefix?: string;
    services?: BluetoothServiceUUID[];
}

interface RequestDeviceOptions {
    filters?: BluetoothLEScanFilter[];
    optionalServices?: BluetoothServiceUUID[];
    acceptAllDevices?: boolean;
}

interface WatchAdvertisementsOptions {
    signal?: AbortSignal;
}

interface BluetoothRemoteGATTCharacteristic extends EventTarget {
    readonly uuid: string;
    readonly value?: DataView;
    startNotifications(): Promise<BluetoothRemoteGATTCharacteristic>;
    stopNotifications(): Promise<BluetoothRemoteGATTCharacteristic>;
    writeValueWithResponse(value: BufferSource): Promise<void>;
}

interface BluetoothRemoteGATTService {
    readonly uuid: string;
    getCharacteristic(characteristic: BluetoothCharacteristicUUID): Promise<BluetoothRemoteGATTCharacteristic>;
}

interface BluetoothRemoteGATTServer {
    readonly device: BluetoothDevice;
    readonly connected: boolean;
    connect(): Promise<BluetoothRemoteGATTServer>;
    disconnect(): void;
    getPrimaryService(service: BluetoothServiceUUID): Promise<BluetoothRemoteGATTService>;
}

interface BluetoothDevice extends EventTarget {
    readonly id: string;
    readonly name?: string;
    readonly gatt?: BluetoothRemoteGATTServer;
    /** Chrome 101+. */
    forget?(): Promise<void>;
    /** Chrome, behind the new permissions backend. */
    watchAdvertisements?(options?: WatchAdvertisementsOptions): Promise<void>;
}

interface Bluetooth extends EventTarget {
    getAvailability(): Promise<boolean>;
    requestDevice(options?: RequestDeviceOptions): Promise<BluetoothDevice>;
    /** Devices this origin was already granted; Chrome with the new permissions backend only. */
    getDevices?(): Promise<BluetoothDevice[]>;
}

interface Navigator {
    readonly bluetooth?: Bluetooth;
}
