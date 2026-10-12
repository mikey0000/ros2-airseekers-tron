import {fireEvent, render, screen} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {TrackedObstaclesPanel} from './TrackedObstaclesPanel.tsx';

const mocks = vi.hoisted(() => ({confirm: vi.fn<(options: {content: string; onOk: () => Promise<void>}) => void>(), success: vi.fn(), error: vi.fn(), callCreate: vi.fn()}));
vi.mock('../../../hooks/useApi.ts', () => ({useApi: () => ({mowglinext: {callCreate: mocks.callCreate}})}));
vi.mock('../../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));
vi.mock('antd', async (original) => ({
    ...await original<typeof import('antd')>(),
    App: {useApp: () => ({modal: {confirm: mocks.confirm}, notification: {success: mocks.success, error: mocks.error}})},
}));

const showPanel = () => render(<TrackedObstaclesPanel
    obstacles={[{id: 42}]} obstacleAreaIndex={{42: null}} areaNames={{}}
    selectedObstacleId={null} onHoverObstacle={() => {}}
/>);

describe('tracked obstacle ignore', () => {
    beforeEach(() => {
        vi.clearAllMocks();
        mocks.callCreate.mockResolvedValue({data: {message: 'removed'}});
    });

    it('allows ignoring outside an area and waits for confirmation', async () => {
        showPanel();
        expect(screen.getByRole('button', {name: 'Promote'})).toBeDisabled();
        fireEvent.click(screen.getByRole('button', {name: 'Ignore'}));
        expect(mocks.callCreate).not.toHaveBeenCalled();
        expect(mocks.confirm.mock.calls[0][0].content).toContain('may reappear');
        await mocks.confirm.mock.calls[0][0].onOk();
        expect(mocks.callCreate).toHaveBeenCalledExactlyOnceWith('ignore_obstacle', {obstacle_id: 42});
        expect(mocks.success).toHaveBeenCalledOnce();
    });

    it('shows backend failures and leaves the dialog retryable', async () => {
        mocks.callCreate.mockRejectedValue({error: {error: 'Obstacle 42 not found'}});
        showPanel();
        fireEvent.click(screen.getByRole('button', {name: 'Ignore'}));
        await expect(mocks.confirm.mock.calls[0][0].onOk()).rejects.toEqual({error: {error: 'Obstacle 42 not found'}});
        expect(mocks.error).toHaveBeenCalledWith(expect.objectContaining({description: 'Obstacle 42 not found'}));
        expect(mocks.success).not.toHaveBeenCalled();
    });
});

describe('tracked obstacle snapshot', () => {
    it('shows the annotated-frame thumbnail served for the obstacle id', () => {
        showPanel();
        const img = screen.getByRole('img', {name: 'Annotated camera frame of obstacle #42'});
        expect(img).toHaveAttribute('src', '/api/mowglinext/obstacles/42/snapshot');
        expect(img.closest('a')).toHaveAttribute('href', '/api/mowglinext/obstacles/42/snapshot');
    });

    it('falls back to the marker icon when the snapshot file is missing', () => {
        showPanel();
        fireEvent.error(screen.getByRole('img', {name: 'Annotated camera frame of obstacle #42'}));
        expect(screen.queryByRole('img', {name: 'Annotated camera frame of obstacle #42'})).not.toBeInTheDocument();
        expect(screen.getByRole('button', {name: 'Promote'})).toBeInTheDocument();
    });

    it('retries a missing snapshot (cache-busted) once the obstacle updates', () => {
        const panel = (count: number) => <TrackedObstaclesPanel
            obstacles={[{id: 42, status: 1, observation_count: count}]} obstacleAreaIndex={{42: null}} areaNames={{}}
            selectedObstacleId={null} onHoverObstacle={() => {}}
        />;
        const name = 'Annotated camera frame of obstacle #42';
        const {rerender} = render(panel(3));
        fireEvent.error(screen.getByRole('img', {name}));
        expect(screen.queryByRole('img', {name})).not.toBeInTheDocument();
        rerender(panel(3));
        expect(screen.queryByRole('img', {name})).not.toBeInTheDocument();
        rerender(panel(4));
        expect(screen.getByRole('img', {name})).toHaveAttribute('src', '/api/mowglinext/obstacles/42/snapshot?v=1%3A4');
    });
});
