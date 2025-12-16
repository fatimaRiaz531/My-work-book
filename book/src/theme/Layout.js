import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatbotWidget from '../components/ChatbotWidget';
import { AuthProvider } from '../components/Auth/AuthProvider';

export default function Layout(props) {
  return (
    <AuthProvider>
      <OriginalLayout {...props} />
      <ChatbotWidget />
    </AuthProvider>
  );
}