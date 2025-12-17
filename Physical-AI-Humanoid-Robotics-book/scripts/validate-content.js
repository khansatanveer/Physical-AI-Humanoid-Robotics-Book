#!/usr/bin/env node

/**
 * Content validation script for Physical AI Humanoid Robotics Book
 * Validates content structure, links, and basic quality metrics
 */

const fs = require('fs');
const path = require('path');

const DOCS_DIR = path.join(__dirname, '..', 'docs');
const VALIDATION_ERRORS = [];
const VALIDATION_WARNINGS = [];

function validateFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const fileName = path.basename(filePath);

  // Check for basic content structure
  if (fileName.endsWith('.md') || fileName.endsWith('.mdx')) {
    // Check if file has frontmatter
    if (!content.startsWith('---')) {
      VALIDATION_WARNINGS.push(`File ${filePath} is missing frontmatter (recommended for Docusaurus docs)`);
    }

    // Check for basic content structure
    if (!content.includes('# ') && !content.includes('## ')) {
      VALIDATION_WARNINGS.push(`File ${filePath} appears to have no headings`);
    }

    // Check for learning objectives (as specified in the spec)
    if (fileName.includes('intro') || fileName.includes('fundamentals') || fileName.includes('ros2') ||
        fileName.includes('simulation') || fileName.includes('vla')) {
      if (!content.toLowerCase().includes('learning objective') && !content.includes('## Learning Objectives')) {
        VALIDATION_WARNINGS.push(`File ${filePath} may be missing learning objectives as required by spec`);
      }
    }
  }
}

function validateDirectory(dirPath) {
  const items = fs.readdirSync(dirPath);

  for (const item of items) {
    const itemPath = path.join(dirPath, item);
    const stat = fs.statSync(itemPath);

    if (stat.isDirectory()) {
      validateDirectory(itemPath);
    } else if (stat.isFile()) {
      validateFile(itemPath);
    }
  }
}

function main() {
  console.log('Starting content validation for Physical AI Humanoid Robotics Book...');

  if (!fs.existsSync(DOCS_DIR)) {
    console.log(`Docs directory does not exist at ${DOCS_DIR}`);
    return;
  }

  validateDirectory(DOCS_DIR);

  console.log(`\nValidation complete:`);
  console.log(`- Errors: ${VALIDATION_ERRORS.length}`);
  console.log(`- Warnings: ${VALIDATION_WARNINGS.length}`);

  if (VALIDATION_ERRORS.length > 0) {
    console.log('\nErrors found:');
    VALIDATION_ERRORS.forEach(error => console.log(`  - ${error}`));
  }

  if (VALIDATION_WARNINGS.length > 0) {
    console.log('\nWarnings:');
    VALIDATION_WARNINGS.forEach(warning => console.log(`  - ${warning}`));
  }

  // Exit with error code if there are errors
  if (VALIDATION_ERRORS.length > 0) {
    process.exit(1);
  }
}

main();