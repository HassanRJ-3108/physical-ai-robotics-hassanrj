import React from 'react';
import { AuthProvider } from '@site/src/lib/AuthContext';
import ChatBot from '@site/src/components/ChatBot';

export default function Root({ children }) {
    return (
        <AuthProvider>
            {children}
            <ChatBot />
        </AuthProvider>
    );
}
