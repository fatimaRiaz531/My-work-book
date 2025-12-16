import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthProvider';
import ChapterControls from '../components/Personalization/ChapterControls';
import SignupForm from '../components/Auth/SignupForm';
import LoginForm from '../components/Auth/LoginForm';
import UserProfile from '../components/Auth/UserProfile';

function TestPageContent() {
  const { user, loading, login, logout } = useAuth();

  const sampleChapterContent = `
# Introduction to ROS2

ROS2 (Robot Operating System 2) is a flexible framework for writing robot software.
It is a collection of tools, libraries, and conventions that aim to simplify the task
of creating complex and robust robot behavior across a wide variety of robot platforms.

## Key Features

- **Communication**: ROS2 uses DDS (Data Distribution Service) for communication between nodes.
- **Architecture**: Client Library implementations in C++, Python, and other languages.
- **Security**: Built-in security features for safe robot operation.
- **Real-time**: Support for real-time systems and deterministic behavior.

## Getting Started

To get started with ROS2, you'll need to install the appropriate distribution
and set up your development environment with the necessary tools and libraries.
`;

  const handleSignupSuccess = (userData) => {
    login(userData);
  };

  const handleLoginSuccess = (userData) => {
    login(userData);
  };

  const handleProfileUpdate = (userData) => {
    login(userData);
  };

  if (loading) {
    return (
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Loading...</h1>
            <p>Please wait while we load the application.</p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="container margin-vert--xl">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <h1>AI-Native Book Features Test</h1>

          <div className="margin-vert--lg">
            <h2>Authentication Status</h2>
            {user ? (
              <div>
                <p>✅ Logged in as: {user.name || user.email}</p>
                <button onClick={logout} className="button button--secondary">
                  Logout
                </button>
              </div>
            ) : (
              <div>
                <p>❌ Not logged in</p>
                <div style={{ display: 'flex', gap: '1rem', marginTop: '1rem' }}>
                  <div>
                    <h3>Signup</h3>
                    <SignupForm onSignupSuccess={handleSignupSuccess} />
                  </div>
                  <div>
                    <h3>Login</h3>
                    <LoginForm onLoginSuccess={handleLoginSuccess} />
                  </div>
                </div>
              </div>
            )}
          </div>

          {user && (
            <div className="margin-vert--lg">
              <h2>User Profile</h2>
              <UserProfile user={user} onProfileUpdate={handleProfileUpdate} />
            </div>
          )}

          <div className="margin-vert--lg">
            <h2>Chapter with Personalization & Translation Controls</h2>
            <ChapterControls chapterTitle="Introduction to ROS2">
              {sampleChapterContent}
            </ChapterControls>
          </div>
        </div>
      </div>
    </div>
  );
}

function TestPage() {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  return (
    <Layout title="AI-Native Book Features Test" description="Test page for all features">
      {isClient ? <TestPageContent /> : (
        <div className="container margin-vert--xl">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <h1>AI-Native Book Features Test</h1>
              <p>Loading application...</p>
            </div>
          </div>
        </div>
      )}
    </Layout>
  );
}

export default TestPage;