import React from 'react';
import {describe, expect, it, vi} from 'vitest';
import {act, render, screen} from '@testing-library/react';
import {createMemoryRouter, RouterProvider} from 'react-router-dom';
import AppShell from './AppShell.tsx';
import {NotFoundPage, RouteErrorPage} from './RouteFallbacks.tsx';
import en from '../i18n/locales/en.json';

// Simulate a background/occluded tab for the whole file: frame callbacks never
// fire. Hoisted so animation libraries capture the stalled scheduler at import.
// The route swap must not depend on any animation completing.
vi.hoisted(() => {
    globalThis.requestAnimationFrame = () => 0;
});

// The shell's live widgets talk to the robot; stub them so the test exercises
// only the routing/outlet behaviour.
vi.mock('./MowerStatus.tsx', () => ({MowerStatus: () => null}));
vi.mock('./NotificationBell.tsx', () => ({NotificationBell: () => null}));
vi.mock('./LanguageSwitcher.tsx', () => ({LanguageSwitcher: () => null}));
vi.mock('./LiveStatusStrip.tsx', () => ({LiveStatusStrip: () => null}));
vi.mock('../hooks/useNotificationCenter.tsx', () => ({useAutoNotifications: () => undefined}));
vi.mock('../hooks/useHighLevelStatus.ts', () => ({useHighLevelStatus: () => ({highLevelStatus: {}})}));
vi.mock('../hooks/useEmergency.ts', () => ({useEmergency: () => ({})}));
vi.mock('../hooks/useStatus.ts', () => ({useStatus: () => ({})}));
vi.mock('../hooks/useHostUpdater', () => ({useHostUpdater: () => ({data: undefined, error: undefined})}));
vi.mock('../hooks/useIsMobile', () => ({useIsMobile: () => false}));

// Settles once the test resolves it, so a lazy page can be held mid-load.
let releaseSlowPage: () => void = () => {};
const SlowPage = React.lazy(() => new Promise<{default: React.ComponentType}>((resolve) => {
    releaseSlowPage = () => resolve({default: () => <p>slow body</p>});
}));

function Boom(): React.ReactNode {
    throw new Error('page exploded');
}

function renderShell(initialPath: string) {
    const router = createMemoryRouter([{
        path: '/',
        element: <AppShell/>,
        errorElement: <RouteErrorPage/>,
        children: [
            ...[
                {path: '/mowglinext', element: <p>home body</p>},
                {path: '/parameters', element: <p>parameters body</p>},
                {path: '/schedule', element: <p>schedule body</p>},
                {path: '/statistics', element: <SlowPage/>},
                {path: '/logs', element: <Boom/>},
            ].map((r) => ({...r, errorElement: <RouteErrorPage/>})),
            {path: '*', element: <NotFoundPage/>},
        ],
    }], {initialEntries: [initialPath]});
    render(<RouterProvider router={router}/>);
    return router;
}

const outlets = () => document.querySelectorAll('main > [data-testid="page-outlet"]');

describe('AppShell outlet', () => {
    it('swaps the page body on navigation even when no frame ever runs', async () => {
        const router = renderShell('/parameters');
        expect(await screen.findByText('parameters body')).toBeInTheDocument();

        await act(() => router.navigate('/schedule'));
        expect(await screen.findByText('schedule body')).toBeInTheDocument();
        expect(screen.queryByText('parameters body')).not.toBeInTheDocument();

        await act(() => router.navigate('/mowglinext'));
        expect(await screen.findByText('home body')).toBeInTheDocument();
        expect(screen.queryByText('schedule body')).not.toBeInTheDocument();
    });

    it('keeps exactly one page wrapper in main, so nothing stacks above the page', async () => {
        const router = renderShell('/mowglinext');
        await screen.findByText('home body');
        expect(outlets()).toHaveLength(1);
        expect(outlets()[0].getAttribute('style')).not.toMatch(/transform|opacity/);

        await act(() => router.navigate('/parameters'));
        await screen.findByText('parameters body');
        expect(outlets()).toHaveLength(1);
    });

    it('suspends a lazy page inside the shell instead of replacing the app', async () => {
        const router = renderShell('/mowglinext');
        await screen.findByText('home body');

        await act(() => router.navigate('/statistics'));
        // The side-rail is still mounted while the page chunk loads.
        expect(screen.getByRole('button', {name: en.nav.home})).toBeInTheDocument();
        expect(screen.queryByText('slow body')).not.toBeInTheDocument();

        act(() => releaseSlowPage());
        expect(await screen.findByText('slow body')).toBeInTheDocument();
    });

    it('renders a not-found page inside the shell for unknown routes', async () => {
        renderShell('/stats');
        expect(await screen.findByText(en.routeError.notFoundTitle)).toBeInTheDocument();
        expect(screen.getByRole('button', {name: en.nav.home})).toBeInTheDocument();
    });

    it('renders a recoverable error page inside the shell when a page throws', async () => {
        const consoleError = vi.spyOn(console, 'error').mockImplementation(() => {});
        renderShell('/logs');
        expect(await screen.findByText(en.routeError.errorTitle)).toBeInTheDocument();
        expect(screen.getByText('page exploded')).toBeInTheDocument();
        expect(screen.getByRole('button', {name: en.routeError.reload})).toBeInTheDocument();
        expect(screen.getByRole('button', {name: en.nav.home})).toBeInTheDocument();
        consoleError.mockRestore();
    });
});
