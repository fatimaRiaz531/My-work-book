import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthProvider';
import PersonalizationButton from './PersonalizationButton';
import TranslationButton from './TranslationButton';

const ChapterControls = ({ chapterTitle, children }) => {
  const { user } = useAuth();
  const [activeTab, setActiveTab] = useState('original'); // 'original', 'personalized', 'translated'

  return (
    <div className="chapter-controls-container">
      <div className="chapter-header">
        <h1>{chapterTitle}</h1>

        <div className="control-buttons">
          {user && (
            <>
              <button
                className={`control-button ${activeTab === 'personalized' ? 'active' : ''}`}
                onClick={() => setActiveTab(activeTab === 'personalized' ? 'original' : 'personalized')}
              >
                {activeTab === 'personalized' ? ' revert personalization' : ' personalize content'}
              </button>

              <button
                className={`control-button ${activeTab === 'translated' ? 'active' : ''}`}
                onClick={() => setActiveTab(activeTab === 'translated' ? 'original' : 'translated')}
              >
                {activeTab === 'translated' ? ' revert translation' : ' translate to Urdu'}
              </button>
            </>
          )}

          {!user && (
            <div className="auth-prompt">
              <p>Please <a href="/auth/login">login</a> to access personalization and translation features</p>
            </div>
          )}
        </div>
      </div>

      <div className="chapter-content">
        {activeTab === 'original' && children}
        {activeTab === 'personalized' && (
          <PersonalizationButton content={children} contentType="chapter">
            {children}
          </PersonalizationButton>
        )}
        {activeTab === 'translated' && (
          <TranslationButton content={children} contentType="chapter">
            {children}
          </TranslationButton>
        )}
      </div>
    </div>
  );
};

export default ChapterControls;